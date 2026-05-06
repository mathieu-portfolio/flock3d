#include <flock3d/experiment/ExperimentRunner.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    std::string error{};
    const auto config = flock3d::experiment::parse_cli(argc, argv, error);
    if (!config.has_value()) {
        std::cerr << "flock3d_experiment_runner: " << error << '\n';
        return 2;
    }

    const auto result = flock3d::experiment::run_experiment(*config);
    std::cout << "wrote " << result.rows_written << " rows to " << result.output_path.string() << '\n';
    return 0;
}
