import argparse
import csv
import math
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.route_planner import RoutePlanner
from omegaconf import OmegaConf


def route_reference(scenario, planning_problem):
    route = RoutePlanner(scenario, planning_problem).plan_routes().retrieve_first_route()
    points = []
    for lanelet_id in route.lanelet_ids:
        lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        for point in lanelet.center_vertices:
            if not points or np.linalg.norm(np.asarray(point) - np.asarray(points[-1])) > 1e-9:
                points.append(np.asarray(point, dtype=float))
    return np.asarray(points, dtype=float)


def project_state(position, orientation, velocity, acceleration, reference):
    position = np.asarray(position, dtype=float)
    a = reference[:-1]
    b = reference[1:]
    ab = b - a
    lengths = np.linalg.norm(ab, axis=1)
    valid = lengths > 1e-9
    ab2 = np.sum(ab * ab, axis=1)
    t = np.zeros(len(ab), dtype=float)
    t[valid] = np.clip(np.sum((position - a[valid]) * ab[valid], axis=1) / ab2[valid], 0.0, 1.0)
    projections = a + t[:, None] * ab
    distances = np.linalg.norm(position - projections, axis=1)
    index = int(np.argmin(distances))
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    s = cumulative[index] + t[index] * lengths[index]
    direction = ab[index] / lengths[index]
    offset = position - projections[index]
    d = direction[0] * offset[1] - direction[1] * offset[0]
    reference_yaw = math.atan2(direction[1], direction[0])
    delta = orientation - reference_yaw
    s_d = velocity * math.cos(delta)
    d_d = velocity * math.sin(delta)
    s_dd = acceleration * math.cos(delta)
    d_dd = acceleration * math.sin(delta)
    return s, s_d, s_dd, d, d_d, d_dd


def polygons_from_shape(shape):
    geometry = shape.shapely_object
    if geometry.geom_type == "Polygon":
        return [np.asarray(geometry.exterior.coords[:-1], dtype=float)]
    if geometry.geom_type == "MultiPolygon":
        return [np.asarray(poly.exterior.coords[:-1], dtype=float) for poly in geometry.geoms]
    return []


def obstacle_frames(scenario, current_time_step, tick_t, max_t, cache=None):
    frame_count = int(math.ceil(max_t / tick_t)) + 1
    frames = []
    if cache is None:
        cache = {}
    for frame in range(frame_count):
        commonroad_step = current_time_step + int(round((frame * tick_t) / scenario.dt))
        if commonroad_step not in cache:
            polygons = []
            for obstacle in scenario.obstacles:
                occupancy = obstacle.occupancy_at_time(commonroad_step)
                if occupancy is None:
                    continue
                polygons.extend(polygons_from_shape(occupancy.shape))
            cache[commonroad_step] = polygons
        frames.append(cache[commonroad_step])
    return frames


def write_input(path, cfg, state, reference, frames, target_speed=None):
    planner = cfg.PLANNER
    vehicle = cfg.VEHICLE
    if target_speed is None:
        target_speed = float(cfg.TARGET_SPEED)
        lowest_speed = float(planner.LOWEST_SPEED)
        highest_speed = float(planner.HIGHEST_SPEED)
    else:
        lowest_speed = max(0.0, target_speed - 2.5)
        highest_speed = min(float(vehicle.MAX_SPEED), target_speed + 2.5)
    with path.open("w", encoding="utf-8") as file:
        file.write(f"SETTINGS {planner.TICK_T} {planner.ROAD_WIDTH} {planner.N_W_SAMPLE} {lowest_speed} {highest_speed} {planner.N_S_SAMPLE} {planner.MIN_T} {planner.MAX_T} {planner.N_T_SAMPLE} {int(planner.CHECK_OBSTACLE)} {int(planner.CHECK_BOUNDARY)}\n")
        file.write(f"VEHICLE {vehicle.LENGTH} {vehicle.WIDTH} {vehicle.MAX_SPEED} {vehicle.MAX_ACCEL} {vehicle.MAX_CURVATURE}\n")
        file.write("STATE " + " ".join(str(value) for value in state) + "\n")
        file.write(f"TARGET_SPEED {target_speed}\n")
        file.write(f"REFERENCE {len(reference)}\n")
        for x, y in reference:
            file.write(f"{x} {y}\n")
        file.write(f"FRAMES {len(frames)}\n")
        for polygons in frames:
            file.write(f"FRAME {len(polygons)}\n")
            for polygon in polygons:
                file.write(f"POLYGON {len(polygon)}\n")
                for x, y in polygon:
                    file.write(f"{x} {y}\n")


def read_best(path):
    rows = []
    with path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            rows.append(
                {
                    "t": float(row["t"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "s": float(row["s"]),
                    "d": float(row["d"]),
                    "speed": float(row["speed"]),
                    "accel": float(row["accel"]),
                    "d_speed": float(row["d_speed"]),
                    "d_accel": float(row["d_accel"]),
                }
            )
    return rows


def trajectory_yaw(rows, index, fallback):
    if len(rows) < 2:
        return fallback
    if index < len(rows) - 1:
        dx = rows[index + 1]["x"] - rows[index]["x"]
        dy = rows[index + 1]["y"] - rows[index]["y"]
    else:
        dx = rows[index]["x"] - rows[index - 1]["x"]
        dy = rows[index]["y"] - rows[index - 1]["y"]
    if abs(dx) + abs(dy) < 1e-9:
        return fallback
    return math.atan2(dy, dx)


def run_planning_cycles(cfg, scenario, planning_problem, reference, executable, output_dir):
    initial_state = planning_problem.initial_state
    current_position = np.asarray(initial_state.position, dtype=float)
    current_orientation = float(initial_state.orientation)
    current_velocity = float(initial_state.velocity)
    current_acceleration = float(initial_state.acceleration)
    current_time_step = int(initial_state.time_step)
    state = project_state(
        current_position,
        current_orientation,
        current_velocity,
        current_acceleration,
        reference,
    )
    tick_t = float(cfg.PLANNER.TICK_T)
    next_index = max(1, int(round(float(scenario.dt) / tick_t)))
    goal_state = planning_problem.goal.state_list[0]
    goal_position = goal_state.position
    goal_shape = goal_position.shapes[0] if hasattr(goal_position, "shapes") else goal_position
    goal_center = goal_shape.shapely_object.centroid
    goal_s = project_state(
        (goal_center.x, goal_center.y), 0.0, 0.0, 0.0, reference
    )[0]
    goal_start_step = int(goal_state.time_step.start)
    end_time_step = int(goal_state.time_step.end)
    executed_states = []
    best_paths = []
    frame_steps = []
    obstacle_cache = {}
    reached_goal = planning_problem.goal.is_reached(initial_state)

    while current_time_step < end_time_step and not reached_goal:
        frames = obstacle_frames(
            scenario,
            current_time_step,
            tick_t,
            float(cfg.PLANNER.MAX_T),
            obstacle_cache,
        )
        input_path = output_dir / "scenario_input.txt"
        remaining_time = max((goal_start_step - current_time_step) * float(scenario.dt), float(scenario.dt))
        target_speed = max(0.0, min(float(cfg.TARGET_SPEED), (goal_s - state[0]) / remaining_time))
        write_input(input_path, cfg, state, reference, frames, target_speed)
        result = subprocess.run(
            [str(executable), str(input_path), str(output_dir)],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            break

        rows = read_best(output_dir / "best_trajectory.csv")
        if len(rows) <= next_index:
            break

        best_paths.append(
            (
                np.asarray([row["x"] for row in rows], dtype=float),
                np.asarray([row["y"] for row in rows], dtype=float),
            )
        )
        frame_steps.append(current_time_step)

        yaw = trajectory_yaw(rows, next_index, current_orientation)
        previous_yaw = current_orientation
        current_position = np.asarray(
            [rows[next_index]["x"], rows[next_index]["y"]],
            dtype=float,
        )
        current_orientation = yaw
        current_velocity = rows[next_index]["speed"]
        current_acceleration = rows[next_index]["accel"]
        state = (
            rows[next_index]["s"],
            rows[next_index]["speed"],
            rows[next_index]["accel"],
            rows[next_index]["d"],
            rows[next_index]["d_speed"],
            rows[next_index]["d_accel"],
        )
        current_time_step += 1
        yaw_rate = (current_orientation - previous_yaw) / float(scenario.dt)

        next_state = InitialState(
            time_step=current_time_step,
            position=current_position,
            orientation=current_orientation,
            velocity=current_velocity,
            acceleration=current_acceleration,
            yaw_rate=yaw_rate,
        )
        executed_states.append(next_state)

        reached_goal = planning_problem.goal.is_reached(next_state)

    return executed_states, best_paths, frame_steps, reached_goal


def create_ego_vehicle(cfg, planning_problem, executed_states):
    shape = Rectangle(
        length=float(cfg.VEHICLE.LENGTH),
        width=float(cfg.VEHICLE.WIDTH),
    )
    if executed_states:
        trajectory = Trajectory(
            initial_time_step=int(executed_states[0].time_step),
            state_list=executed_states,
        )
        prediction = TrajectoryPrediction(
            trajectory=trajectory,
            shape=shape,
        )
    else:
        prediction = None
    return DynamicObstacle(
        obstacle_id=100,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=shape,
        initial_state=planning_problem.initial_state,
        prediction=prediction,
    )


def goal_shapes(planning_problem):
    for goal_state in planning_problem.goal.state_list:
        if not hasattr(goal_state, "position"):
            continue
        position = goal_state.position
        positions = position if isinstance(position, list) else [position]
        for item in positions:
            yield from item.shapes if hasattr(item, "shapes") else [item]


def plot_limits(planning_problem, executed_states):
    x = [float(planning_problem.initial_state.position[0])]
    y = [float(planning_problem.initial_state.position[1])]
    x.extend(float(state.position[0]) for state in executed_states)
    y.extend(float(state.position[1]) for state in executed_states)
    for shape in goal_shapes(planning_problem):
        left, bottom, right, top = shape.shapely_object.bounds
        x.extend((left, right))
        y.extend((bottom, top))
    x_min = min(x) - 30.0
    x_max = max(x) + 30.0
    y_min = min(y) - 30.0
    y_max = max(y) + 30.0
    length = max(x_max - x_min, y_max - y_min)
    if length == x_max - x_min:
        return (
            x_min,
            x_max,
            y_min - (length - (y_max - y_min)) / 2.0,
            y_max + (length - (y_max - y_min)) / 2.0,
        )
    return (
        x_min - (length - (x_max - x_min)) / 2.0,
        x_max + (length - (x_max - x_min)) / 2.0,
        y_min,
        y_max,
    )


def save_gif(cfg, scenario, planning_problem, ego_vehicle, executed_states, best_paths, frame_steps, output_path):
    cache_dir = output_path.parent / "gif_cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    limits = plot_limits(planning_problem, executed_states)
    images = []

    for index, (path, time_step) in enumerate(zip(best_paths, frame_steps)):
        fig = plt.figure(figsize=(10, 6))
        renderer = MPRenderer()
        renderer.draw_params.time_begin = time_step
        renderer.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
        renderer.draw_params.dynamic_obstacle.occupancy.draw_occupancies = False
        renderer.draw_params.lanelet_network.traffic_light.draw_traffic_lights = False
        renderer.draw_params.lanelet_network.traffic_sign.draw_traffic_signs = False
        renderer.draw_params.planning_problem.initial_state.state.draw_arrow = False
        scenario.draw(renderer, renderer.draw_params)
        planning_problem.goal.draw(renderer, renderer.draw_params.planning_problem.goal_region)
        renderer.draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "g"
        ego_vehicle.draw(renderer)
        renderer.render()

        x, y = path
        renderer.ax.plot(x[1:], y[1:], color="red", linewidth=3.0, zorder=25)
        renderer.ax.set_xlim(limits[0], limits[1])
        renderer.ax.set_ylim(limits[2], limits[3])
        renderer.ax.set_aspect("equal")
        renderer.ax.set_title(f"FOP planning cycle {index}, time step {time_step}")

        frame_path = cache_dir / f"{index}.jpg"
        fig.savefig(frame_path, dpi=100, bbox_inches="tight")
        plt.close(fig)
        with Image.open(frame_path) as image:
            images.append(image.convert("RGB").quantize(colors=128))

    if images:
        duration = max(40, int(round(float(scenario.dt) * 1000.0)))
        images[0].save(
            output_path,
            save_all=True,
            append_images=images[1:],
            optimize=True,
            duration=duration,
            loop=0,
        )


def ensure_build(build_dir):
    executable = build_dir / "fop_scenario"
    if not executable.exists():
        subprocess.run(
            ["cmake", "-S", ".", "-B", str(build_dir), "-DCMAKE_BUILD_TYPE=Release"],
            check=True,
            capture_output=True,
            text=True,
        )
    subprocess.run(
        ["cmake", "--build", str(build_dir), "-j"],
        check=True,
        capture_output=True,
        text=True,
    )
    return executable


def run_file(cfg, scenario_path, executable):
    scenario, planning_problem_set = CommonRoadFileReader(str(scenario_path)).open()
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))
    reference = route_reference(scenario, planning_problem)
    output_dir = Path(cfg.OUTPUT_DIR) / scenario_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    executed_states, best_paths, frame_steps, reached_goal = run_planning_cycles(
        cfg,
        scenario,
        planning_problem,
        reference,
        executable,
        output_dir,
    )
    print(f"{scenario_path.stem}: {'goal reach' if reached_goal else 'planning fail'}", flush=True)

    if cfg.SAVE_PLOT and best_paths:
        ego_vehicle = create_ego_vehicle(cfg, planning_problem, executed_states)
        save_gif(
            cfg,
            scenario,
            planning_problem,
            ego_vehicle,
            executed_states,
            best_paths,
            frame_steps,
            output_dir / f"{scenario_path.stem}.gif",
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="cfgs/demo_config.yaml")
    args = parser.parse_args()
    cfg = OmegaConf.load(args.config)
    build_dir = Path(cfg.BUILD_DIR)
    executable = ensure_build(build_dir)
    input_dir = Path(cfg.INPUT_DIR)
    files = list(cfg.FILES or [])
    if not files:
        files = sorted(path.name for path in input_dir.glob("*.xml"))
    for filename in files:
        run_file(cfg, input_dir / filename, executable)


if __name__ == "__main__":
    main()
