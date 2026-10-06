#include "fop/scenario_io.h"

#include <fstream>
#include <stdexcept>
#include <string>

namespace fop {
namespace {

void expect(std::istream& stream, const std::string& expected) {
    std::string value;
    stream >> value;
    if (!stream || value != expected) {
        throw std::runtime_error("invalid scenario input");
    }
}

}

ScenarioInput loadScenarioInput(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open scenario input");
    }

    ScenarioInput input;
    int check_obstacle = 0;
    int check_boundary = 0;

    expect(file, "SETTINGS");
    file >> input.settings.tick_t
         >> input.settings.road_width
         >> input.settings.num_width
         >> input.settings.lowest_speed
         >> input.settings.highest_speed
         >> input.settings.num_speed
         >> input.settings.min_t
         >> input.settings.max_t
         >> input.settings.num_t
         >> check_obstacle
         >> check_boundary;
    input.settings.check_obstacle = check_obstacle != 0;
    input.settings.check_boundary = check_boundary != 0;

    expect(file, "VEHICLE");
    file >> input.vehicle.length
         >> input.vehicle.width
         >> input.vehicle.max_speed
         >> input.vehicle.max_accel
         >> input.vehicle.max_curvature;

    expect(file, "STATE");
    file >> input.state.s
         >> input.state.s_d
         >> input.state.s_dd
         >> input.state.d
         >> input.state.d_d
         >> input.state.d_dd;

    expect(file, "TARGET_SPEED");
    file >> input.target_speed;

    expect(file, "REFERENCE");
    std::size_t reference_count = 0;
    file >> reference_count;
    input.reference_x.resize(reference_count);
    input.reference_y.resize(reference_count);
    for (std::size_t i = 0; i < reference_count; ++i) {
        file >> input.reference_x[i] >> input.reference_y[i];
    }

    expect(file, "FRAMES");
    std::size_t frame_count = 0;
    file >> frame_count;
    input.obstacles.resize(frame_count);
    for (std::size_t frame = 0; frame < frame_count; ++frame) {
        expect(file, "FRAME");
        std::size_t polygon_count = 0;
        file >> polygon_count;
        input.obstacles[frame].resize(polygon_count);
        for (std::size_t polygon = 0; polygon < polygon_count; ++polygon) {
            expect(file, "POLYGON");
            std::size_t vertex_count = 0;
            file >> vertex_count;
            input.obstacles[frame][polygon].resize(vertex_count);
            for (std::size_t vertex = 0; vertex < vertex_count; ++vertex) {
                file >> input.obstacles[frame][polygon][vertex].x
                     >> input.obstacles[frame][polygon][vertex].y;
            }
        }
    }

    if (!file) {
        throw std::runtime_error("invalid scenario input");
    }
    if (input.reference_x.size() < 2) {
        throw std::runtime_error("reference path is too short");
    }

    return input;
}

}
