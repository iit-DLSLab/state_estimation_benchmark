#pragma once
#include "benchmark/Types.hpp"
#include <string>

namespace benchmark {

MeasurementSequence loadMeasurements(const std::string& csv_path, int num_joints = 12);
GroundTruthSequence loadGroundTruth(const std::string& csv_path);

} // namespace benchmark
