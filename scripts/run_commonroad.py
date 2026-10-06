import argparse
import csv
import math
import subprocess
from io import BytesIO
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from commonroad.common.file_reader import CommonRoadFileReader
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


def project_state(initial_state, reference):
    position = np.asarray(initial_state.position, dtype=float)
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
    orientation = float(getattr(initial_state, "orientation", reference_yaw))
    velocity = float(getattr(initial_state, "velocity", 0.0))
    acceleration = float(getattr(initial_state, "acceleration", 0.0))
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


def obstacle_frames(scenario, initial_time_step, tick_t, max_t):
    frame_count = int(math.ceil(max_t / tick_t)) + 1
    frames = []
    for frame in range(frame_count):
        commonroad_step = initial_time_step + int(round((frame * tick_t) / scenario.dt))
        polygons = []
        for obstacle in scenario.obstacles:
            occupancy = obstacle.occupancy_at_time(commonroad_step)
            if occupancy is None:
                continue
            polygons.extend(polygons_from_shape(occupancy.shape))
        frames.append(polygons)
    return frames


def write_input(path, cfg, state, reference, frames):
    planner = cfg.PLANNER
    vehicle = cfg.VEHICLE
    with path.open("w", encoding="utf-8") as file:
        file.write(f"SETTINGS {planner.TICK_T} {planner.ROAD_WIDTH} {planner.N_W_SAMPLE} {planner.LOWEST_SPEED} {planner.HIGHEST_SPEED} {planner.N_S_SAMPLE} {planner.MIN_T} {planner.MAX_T} {planner.N_T_SAMPLE} {int(planner.CHECK_OBSTACLE)} {int(planner.CHECK_BOUNDARY)}\n")
        file.write(f"VEHICLE {vehicle.LENGTH} {vehicle.WIDTH} {vehicle.MAX_SPEED} {vehicle.MAX_ACCEL} {vehicle.MAX_CURVATURE}\n")
        file.write("STATE " + " ".join(str(value) for value in state) + "\n")
        file.write(f"TARGET_SPEED {cfg.TARGET_SPEED}\n")
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
    t = []
    x = []
    y = []
    with path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            t.append(float(row["t"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
    return np.asarray(t), np.asarray(x), np.asarray(y)


def save_gif(scenario, planning_problem, best_path, output_path, initial_time_step, tick_t):
    t, x, y = read_best(best_path)
    frames = []
    for index in range(len(t)):
        fig, ax = plt.subplots(figsize=(10, 6))
        renderer = MPRenderer(ax=ax)
        renderer.draw_params.time_begin = initial_time_step + int(round(t[index] / scenario.dt))
        scenario.draw(renderer)
        planning_problem.draw(renderer)
        renderer.render()
        ax.plot(x, y, linewidth=1.5)
        ax.plot(x[:index + 1], y[:index + 1], linewidth=3.0)
        ax.scatter([x[index]], [y[index]], s=40)
        ax.axis("equal")
        fig.tight_layout()
        buffer = BytesIO()
        fig.savefig(buffer, format="png", dpi=120)
        plt.close(fig)
        buffer.seek(0)
        with Image.open(buffer) as image:
            frames.append(image.convert("RGB"))
    if frames:
        duration = max(40, int(round(tick_t * 1000.0)))
        frames[0].save(output_path, save_all=True, append_images=frames[1:], duration=duration, loop=0)


def ensure_build(build_dir):
    executable = build_dir / "fop_scenario"
    if executable.exists():
        return executable
    subprocess.run(["cmake", "-S", ".", "-B", str(build_dir), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "-j"], check=True)
    return executable


def run_file(cfg, scenario_path, executable):
    scenario, planning_problem_set = CommonRoadFileReader(str(scenario_path)).open()
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))
    reference = route_reference(scenario, planning_problem)
    initial_state = planning_problem.initial_state
    state = project_state(initial_state, reference)
    frames = obstacle_frames(scenario, int(initial_state.time_step), float(cfg.PLANNER.TICK_T), float(cfg.PLANNER.MAX_T))
    output_dir = Path(cfg.OUTPUT_DIR) / scenario_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    input_path = output_dir / "scenario_input.txt"
    write_input(input_path, cfg, state, reference, frames)
    subprocess.run([str(executable), str(input_path), str(output_dir)], check=True)
    if cfg.SAVE_PLOT:
        save_gif(
            scenario,
            planning_problem,
            output_dir / "best_trajectory.csv",
            output_dir / "result.gif",
            int(initial_state.time_step),
            float(cfg.PLANNER.TICK_T),
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="cfgs/demo_config.yaml")
    args = parser.parse_args()
    cfg = OmegaConf.load(args.config)
    build_dir = Path(cfg.BUILD_DIR)
    executable = ensure_build(build_dir)
    input_dir = Path(cfg.INPUT_DIR)
    files = list(cfg.FILES)
    if not files:
        files = sorted(path.name for path in input_dir.glob("*.xml"))
    for filename in files:
        print(f"scenario: {filename}")
        run_file(cfg, input_dir / filename, executable)


if __name__ == "__main__":
    main()
