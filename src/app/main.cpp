#include "analysis/trajectory_metrics.h"
#include "odometry/landmark_odometry_solver.h"
#include "pipeline/synthetic_pipeline.h"
#include "plotting/octave_plotter.h"
#include "simulation/synthetic_data.h"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct TuningStats {
    EstimatorScales scales;
    DeviationStats gtsam;
    DeviationStats ekf;
};

std::vector<unsigned int> parseSeedList(const char* env_name) {
    const char* value = std::getenv(env_name);
    if (!value || *value == '\0') {
        return {};
    }
    std::vector<unsigned int> seeds;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            continue;
        }
        size_t idx = 0;
        const unsigned long seed = std::stoul(token, &idx, 10);
        if (idx != token.size()) {
            throw std::runtime_error("Invalid seed list value for " + std::string(env_name));
        }
        seeds.push_back(static_cast<unsigned int>(seed));
    }
    return seeds;
}

TuningStats evaluateScales(const SyntheticData& data,
                           const SwerveParams& params,
                           const std::vector<gtsam::Point3>& tag_positions,
                           const MeasurementNoise& noise,
                           const EstimatorScales& scales) {
    const SyntheticRunOutputs outputs =
        runGtsamFromSynthetic(data, params, tag_positions, noise, scales);
    TuningStats out;
    out.scales = scales;
    out.gtsam = computeDeviationStats(outputs.ground_truth, outputs.gtsam);
    out.ekf = computeDeviationStats(outputs.ground_truth, outputs.ekf);
    return out;
}

EstimatorScales tuneGtsamScales(const SyntheticData& data,
                                const SwerveParams& params,
                                const std::vector<gtsam::Point3>& tag_positions,
                                const MeasurementNoise& noise,
                                const EstimatorScales& initial,
                                const std::vector<double>& candidates,
                                const std::vector<double>& vision_candidates) {
    EstimatorScales best = initial;
    double best_mean =
        evaluateScales(data, params, tag_positions, noise, best).gtsam.mean_m;

    for (double candidate : candidates) {
        EstimatorScales trial = best;
        trial.gtsam_wheel_sigma_scale = candidate;
        const double mean =
            evaluateScales(data, params, tag_positions, noise, trial).gtsam.mean_m;
        if (mean < best_mean) {
            best = trial;
            best_mean = mean;
        }
    }
    for (double candidate : vision_candidates) {
        EstimatorScales trial = best;
        trial.gtsam_vision_sigma_scale = candidate;
        const double mean =
            evaluateScales(data, params, tag_positions, noise, trial).gtsam.mean_m;
        if (mean <= best_mean) {
            best = trial;
            best_mean = mean;
        } else {
            break;
        }
    }
    for (double candidate : candidates) {
        EstimatorScales trial = best;
        trial.gtsam_gyro_sigma_scale = candidate;
        const double mean =
            evaluateScales(data, params, tag_positions, noise, trial).gtsam.mean_m;
        if (mean < best_mean) {
            best = trial;
            best_mean = mean;
        }
    }
    return best;
}

EstimatorScales tuneEkfScales(const SyntheticData& data,
                              const SwerveParams& params,
                              const std::vector<gtsam::Point3>& tag_positions,
                              const MeasurementNoise& noise,
                              const EstimatorScales& initial,
                              const std::vector<double>& candidates,
                              const std::vector<double>& vision_candidates) {
    EstimatorScales best = initial;
    double best_mean =
        evaluateScales(data, params, tag_positions, noise, best).ekf.mean_m;

    for (double candidate : candidates) {
        EstimatorScales trial = best;
        trial.ekf_process_sigma_scale = candidate;
        const double mean =
            evaluateScales(data, params, tag_positions, noise, trial).ekf.mean_m;
        if (mean < best_mean) {
            best = trial;
            best_mean = mean;
        }
    }
    for (double candidate : vision_candidates) {
        EstimatorScales trial = best;
        trial.ekf_measurement_sigma_scale = candidate;
        const double mean =
            evaluateScales(data, params, tag_positions, noise, trial).ekf.mean_m;
        if (mean <= best_mean) {
            best = trial;
            best_mean = mean;
        } else {
            break;
        }
    }
    return best;
}

EstimatorScales tuneScales(const SyntheticData& data,
                           const SwerveParams& params,
                           const std::vector<gtsam::Point3>& tag_positions,
                           const MeasurementNoise& noise) {
    const std::vector<double> candidates = {0.5, 1.0, 2.0};
    const std::vector<double> vision_candidates = {
        1.0, 0.75, 0.5, 0.35, 0.25, 0.2, 0.15, 0.1, 0.075, 0.05, 0.035, 0.025, 0.02, 0.015, 0.01
    };
    EstimatorScales scales;
    scales = tuneGtsamScales(data, params, tag_positions, noise, scales, candidates,
                             vision_candidates);
    scales = tuneEkfScales(data, params, tag_positions, noise, scales, candidates,
                           vision_candidates);
    return scales;
}

}

int main() {
    try {
        SwerveParams params;

        std::cout << std::fixed << std::setprecision(6);

        std::string script_path = "src/plotting/plot_trajectories.m";
        if (!std::filesystem::exists(script_path)) {
            const std::string alt = "../src/plotting/plot_trajectories.m";
            if (std::filesystem::exists(alt)) {
                script_path = alt;
            }
        }

        auto runScenario = [&](const std::string& label, const SyntheticConfig& cfg) -> bool {
            std::cout << "\n=== Scenario: " << label << " ===\n";
            const SyntheticData data = generateSyntheticData(cfg, params);
            if (data.samples.size() < 2) {
                std::cerr << "Not enough synthetic samples generated.\n";
                return false;
            }

            MeasurementNoise noise;
            noise.wheel_delta_sigma_m = data.sigmas.wheel_delta_m;
            noise.wheel_angle_sigma_rad = data.sigmas.wheel_angle_rad;
            noise.vision_range_sigma_m = data.sigmas.vision_range_m;
            noise.vision_bearing_sigma_rad = data.sigmas.vision_bearing_rad;
            noise.loop_translation_sigma_m = data.sigmas.loop_translation_m;
            noise.loop_rotation_sigma_rad = data.sigmas.loop_rotation_rad;
            noise.tag_prior_sigma_m = 0.1;
            noise.gyro_delta_sigma_rad = data.sigmas.gyro_delta_rad;
            noise.gyro_drift_rate_per_s = cfg.noise.gyro_drift_rate_per_s.stddev;
            noise.pitch_delta_sigma_rad = data.sigmas.pitch_delta_rad;
            noise.pitch_drift_rate_per_s = cfg.noise.pitch_drift_rate_per_s.stddev;

            const EstimatorScales tuned_scales =
                tuneScales(data, params, cfg.tag_positions, noise);
            const SyntheticRunOutputs outputs =
                runGtsamFromSynthetic(data, params, cfg.tag_positions, noise, tuned_scales);

            const DeviationStats deviation =
                computeDeviationStats(outputs.ground_truth, outputs.gtsam);
            const DeviationStats ekf_deviation =
                computeDeviationStats(outputs.ground_truth, outputs.ekf);

            std::cout << "=== Noise Statistics (mean, variance) ===\n";
            std::cout << "Wheel delta (m): "
                      << data.noise_stats.wheel_delta_m.mean << ", "
                      << data.noise_stats.wheel_delta_m.variance << "\n";
            std::cout << "Wheel angle (rad): "
                      << data.noise_stats.wheel_angle_rad.mean << ", "
                      << data.noise_stats.wheel_angle_rad.variance << "\n";
            std::cout << "Vision range (m): "
                      << data.noise_stats.vision_range_m.mean << ", "
                      << data.noise_stats.vision_range_m.variance << "\n";
            std::cout << "Vision bearing (rad): "
                      << data.noise_stats.vision_bearing_rad.mean << ", "
                      << data.noise_stats.vision_bearing_rad.variance << "\n";
            std::cout << "Vision pixel (px): "
                      << data.noise_stats.vision_pixel_px.mean << ", "
                      << data.noise_stats.vision_pixel_px.variance << "\n";
            if (!data.loop_closures.empty()) {
                std::cout << "Loop translation (m): "
                          << data.noise_stats.loop_translation_m.mean << ", "
                          << data.noise_stats.loop_translation_m.variance << "\n";
                std::cout << "Loop rotation (rad): "
                          << data.noise_stats.loop_rotation_rad.mean << ", "
                          << data.noise_stats.loop_rotation_rad.variance << "\n";
                std::cout << "Loop closures: " << data.loop_closures.size() << "\n";
            }
            std::cout << "Gyro delta (rad): "
                      << data.noise_stats.gyro_delta_rad.mean << ", "
                      << data.noise_stats.gyro_delta_rad.variance << "\n";
            std::cout << "Pitch delta (rad): "
                      << data.noise_stats.pitch_delta_rad.mean << ", "
                      << data.noise_stats.pitch_delta_rad.variance << "\n";

            std::cout << "\n=== Tuned Weights ===\n";
            std::cout << "GTSAM wheel scale: " << tuned_scales.gtsam_wheel_sigma_scale << "\n";
            std::cout << "GTSAM vision scale: " << tuned_scales.gtsam_vision_sigma_scale << "\n";
            std::cout << "GTSAM gyro scale: " << tuned_scales.gtsam_gyro_sigma_scale << "\n";
            std::cout << "EKF process scale: " << tuned_scales.ekf_process_sigma_scale << "\n";
            std::cout << "EKF measurement scale: " << tuned_scales.ekf_measurement_sigma_scale
                      << "\n";

            std::cout << "\n=== Trajectory Deviation (m) ===\n";
            std::cout << "GTSAM max deviation: " << deviation.max_m << "\n";
            std::cout << "GTSAM min deviation: " << deviation.min_m << "\n";
            std::cout << "GTSAM mean deviation: " << deviation.mean_m << "\n";
            std::cout << "EKF max deviation: " << ekf_deviation.max_m << "\n";
            std::cout << "EKF min deviation: " << ekf_deviation.min_m << "\n";
            std::cout << "EKF mean deviation: " << ekf_deviation.mean_m << "\n";

            const std::string output_dir =
                (std::filesystem::path("output") / label).string();
            std::vector<bool> tag_used(cfg.tag_positions.size(), false);
            for (const auto& sample : data.samples) {
                for (const auto& measurement : sample.vision_measurements) {
                    if (measurement.tag_index < tag_used.size()) {
                        tag_used[measurement.tag_index] = true;
                    }
                }
            }
            std::vector<gtsam::Point3> plot_tags;
            for (size_t i = 0; i < tag_used.size(); ++i) {
                if (tag_used[i]) {
                    plot_tags.push_back(cfg.tag_positions[i]);
                }
            }
            if (plot_tags.empty()) {
                plot_tags = cfg.tag_positions;
            }
            const PlotOutputs plot_outputs =
                writePlotInputs(outputs.ground_truth,
                                outputs.wheel_only,
                                outputs.vision_only,
                                outputs.ekf,
                                outputs.gtsam,
                                plot_tags,
                                output_dir);

            if (runOctavePlot(plot_outputs, script_path)) {
                std::cout << "\nPlots saved to:\n"
                          << "  " << plot_outputs.plot_png << "\n"
                          << "  " << plot_outputs.plot_with_raw_png << "\n"
                          << "  " << plot_outputs.yaw_plot_png << "\n";
            } else {
                std::cout << "\nPlotting skipped. CSV outputs:\n"
                          << "  " << plot_outputs.ground_truth_csv << "\n"
                          << "  " << plot_outputs.wheel_only_csv << "\n"
                          << "  " << plot_outputs.vision_csv << "\n"
                          << "  " << plot_outputs.ekf_csv << "\n"
                          << "  " << plot_outputs.tag_csv << "\n"
                          << "  " << plot_outputs.fused_csv << "\n";
            }

            return true;
        };

        SyntheticConfig base_cfg;
        base_cfg.duration_s = 5.0;
        base_cfg.sample_rate_hz = 50.0;
        base_cfg.tag_positions = {
            gtsam::Point3(0.0, 2.0, 1.0)
        };
        base_cfg.robot_z_m = 0.0;
        base_cfg.noise.wheel_delta_m = NoiseSpec{0.0, 0.003};
        base_cfg.noise.wheel_angle_rad = NoiseSpec{0.0, 0.006};
        base_cfg.noise.vision_range_m = NoiseSpec{0.0, 0.08};
        base_cfg.noise.vision_pixel_px = NoiseSpec{0.0, 0.5};
        base_cfg.noise.loop_translation_m = NoiseSpec{0.0, 0.15};
        base_cfg.noise.loop_rotation_rad = NoiseSpec{0.0, 0.1};
        base_cfg.noise.gyro_delta_rad = NoiseSpec{0.0, 0.002};
        base_cfg.noise.gyro_drift_rate_per_s = NoiseSpec{0.0, 0.0002};
        base_cfg.noise.pitch_delta_rad = NoiseSpec{0.0, 0.0};
        base_cfg.noise.pitch_drift_rate_per_s = NoiseSpec{0.0, 0.0};
        base_cfg.camera.image_width_px = 1280;
        base_cfg.camera.fov_deg = 70.0;
        base_cfg.camera.principal_x_px = 640.0;
        base_cfg.seed = 42;

        SyntheticConfig complex_cfg = base_cfg;
        complex_cfg.duration_s = 30.0;
        complex_cfg.trajectory = TrajectoryProfile::Complex;
        complex_cfg.seed = 1337;

        const std::vector<unsigned int> simple_seeds = parseSeedList("GSTAM_SIMPLE_SEEDS");
        const std::vector<unsigned int> complex_seeds = parseSeedList("GSTAM_COMPLEX_SEEDS");

        bool ok = true;
        if (!simple_seeds.empty() || !complex_seeds.empty()) {
            for (unsigned int seed : simple_seeds) {
                SyntheticConfig cfg = base_cfg;
                cfg.seed = seed;
                ok = runScenario("simple_seed" + std::to_string(seed), cfg) && ok;
            }
            for (unsigned int seed : complex_seeds) {
                SyntheticConfig cfg = complex_cfg;
                cfg.seed = seed;
                ok = runScenario("complex_seed" + std::to_string(seed), cfg) && ok;
            }
        } else {
            ok = runScenario("simple", base_cfg) && ok;
            ok = runScenario("complex", complex_cfg) && ok;
        }
        return ok ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
