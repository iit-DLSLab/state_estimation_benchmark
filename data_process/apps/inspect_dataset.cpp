#include "benchmark/DatasetIO.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    std::string root = "data/anymalD_grandtour";
    if (argc > 1) root = argv[1];

    const std::string sensor_csv = root + "/sensor_data.csv";
    const std::string gt_csv     = root + "/groundtruth.csv";

    try {
        auto meas = benchmark::loadMeasurements(sensor_csv, 12); // cambia num_joints se diverso
        auto gt   = benchmark::loadGroundTruth(gt_csv);

        std::cout << "Loaded measurements: " << meas.size() << "\n";
        std::cout << "Loaded GT:           " << gt.size()   << "\n";

        if (!meas.empty()) {
            std::cout << "First measurement t = " << meas.front().t
                      << " omega = " << meas.front().omega.transpose() << "\n";
        }
        if (!gt.empty()) {
            std::cout << "First GT t = " << gt.front().t
                      << " pos = " << gt.front().p.transpose() << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
