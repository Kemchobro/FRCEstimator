#include "plotting/octave_plotter.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {

void writeTrackCsv(const Track2D& track, const std::string& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to open output CSV: " + path);
    }
    out << "timestamp,x,y,theta\n";
    const size_t n = std::min(track.t.size(), track.p.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& pose = track.p[i];
        out << track.t[i] << "," << pose.translation().x() << ","
            << pose.translation().y() << "," << pose.rotation().yaw() << "\n";
    }
}

void writeTagCsv(const std::vector<gtsam::Point3>& tag_positions,
                 const std::string& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to open output CSV: " + path);
    }
    out << "timestamp,x,y,theta\n";
    for (const auto& tag_pos : tag_positions) {
        out << 0.0 << "," << tag_pos.x() << "," << tag_pos.y() << "," << 0.0 << "\n";
    }
}

enum class PlotKind {
    FusedOnly,
    WithRaw,
    Yaw
};

bool runGnuplotPlot(const PlotOutputs& outputs, PlotKind kind) {
    const int has_gnuplot =
        std::system("gnuplot --version > /dev/null 2>&1");
    if (has_gnuplot != 0) {
        return false;
    }

    const std::filesystem::path output_dir =
        std::filesystem::path(outputs.plot_png).parent_path();
    const std::string script_name =
        (kind == PlotKind::WithRaw) ? "plot_trajectories_with_raw.gp" :
        (kind == PlotKind::Yaw) ? "plot_yaw_headings.gp" :
        "plot_trajectories.gp";
    const std::filesystem::path script_path = output_dir / script_name;

    std::ofstream script(script_path.string());
    if (!script) {
        return false;
    }

    script << "set terminal png size 1200,800\n";
    if (kind == PlotKind::WithRaw) {
        script << "set output '" << outputs.plot_with_raw_png << "'\n";
    } else if (kind == PlotKind::Yaw) {
        script << "set output '" << outputs.yaw_plot_png << "'\n";
    } else {
        script << "set output '" << outputs.plot_png << "'\n";
    }
    script << "set datafile separator ','\n";
    script << "set key left top\n";
    if (kind == PlotKind::WithRaw) {
        script << "set xlabel 'X (m)'\n";
        script << "set ylabel 'Y (m)'\n";
        script << "set title 'Ground Truth vs GTSAM vs EKF vs Wheel-only vs Vision vs Tags'\n";
        script << "set size ratio -1\n";
    } else if (kind == PlotKind::Yaw) {
        script << "set xlabel 'Time (s)'\n";
        script << "set ylabel 'Yaw (rad)'\n";
        script << "set title 'Yaw Heading vs Time'\n";
    } else {
        script << "set xlabel 'X (m)'\n";
        script << "set ylabel 'Y (m)'\n";
        script << "set title 'Ground Truth vs GTSAM vs EKF vs Tags'\n";
        script << "set size ratio -1\n";
    }
    if (kind == PlotKind::Yaw) {
        script << "plot '" << outputs.ground_truth_csv
               << "' using 1:4 with lines lw 2 lc rgb '#000000' title 'Ground truth', \\\n";
        script << "     '" << outputs.fused_csv
               << "' using 1:4 with lines lw 2 lc rgb '#cc0000' title 'GTSAM', \\\n";
        script << "     '" << outputs.ekf_csv
               << "' using 1:4 with lines lw 2 lc rgb '#8a2be2' title 'EKF'\n";
    } else {
        script << "plot '" << outputs.ground_truth_csv
               << "' using 2:3 with lines lw 2 lc rgb '#000000' title 'Ground truth', \\\n";
        script << "     '" << outputs.fused_csv
               << "' using 2:3 with lines lw 2 lc rgb '#cc0000' title 'GTSAM', \\\n";
        script << "     '" << outputs.ekf_csv
               << "' using 2:3 with lines lw 2 lc rgb '#8a2be2' title 'EKF', \\\n";
        script << "     '" << outputs.tag_csv
               << "' using 2:3 with points pt 7 ps 1.2 lc rgb '#ffa500' title 'Tags'";
        if (kind == PlotKind::WithRaw) {
            script << ", \\\n     '" << outputs.wheel_only_csv
                   << "' using 2:3 with lines lw 1 lc rgb '#3366cc' title 'Wheel-only', \\\n";
            script << "     '" << outputs.vision_csv
                   << "' using 2:3 with points pt 7 ps 0.5 lc rgb '#228b22' title 'Vision'\n";
        } else {
            script << "\n";
        }
    }
    script.close();

    std::ostringstream cmd;
    cmd << "gnuplot \"" << script_path.string() << "\"";
    const int result = std::system(cmd.str().c_str());
    return result == 0;
}

}

PlotOutputs writePlotInputs(const Track2D& ground_truth,
                            const Track2D& wheel_only,
                            const Track2D& vision_only,
                            const Track2D& ekf_track,
                            const Track2D& fused,
                            const std::vector<gtsam::Point3>& tag_positions,
                            const std::string& output_dir) {
    std::filesystem::create_directories(output_dir);

    PlotOutputs outputs;
    outputs.ground_truth_csv =
        (std::filesystem::path(output_dir) / "ground_truth.csv").string();
    outputs.wheel_only_csv =
        (std::filesystem::path(output_dir) / "wheel_only.csv").string();
    outputs.vision_csv =
        (std::filesystem::path(output_dir) / "vision_only.csv").string();
    outputs.ekf_csv =
        (std::filesystem::path(output_dir) / "ekf.csv").string();
    outputs.tag_csv =
        (std::filesystem::path(output_dir) / "tag.csv").string();
    outputs.fused_csv =
        (std::filesystem::path(output_dir) / "fused.csv").string();
    outputs.plot_png =
        (std::filesystem::path(output_dir) / "trajectory_plot.png").string();
    outputs.plot_with_raw_png =
        (std::filesystem::path(output_dir) / "trajectory_plot_with_raw.png").string();
    outputs.yaw_plot_png =
        (std::filesystem::path(output_dir) / "yaw_plot.png").string();

    writeTrackCsv(ground_truth, outputs.ground_truth_csv);
    writeTrackCsv(wheel_only, outputs.wheel_only_csv);
    writeTrackCsv(vision_only, outputs.vision_csv);
    writeTrackCsv(ekf_track, outputs.ekf_csv);
    writeTagCsv(tag_positions, outputs.tag_csv);
    writeTrackCsv(fused, outputs.fused_csv);

    return outputs;
}

bool runOctaveCommand(const std::string& script_dir,
                      const std::string& function_name,
                      const std::string& args) {
    std::ostringstream cmd;
    cmd << "octave --silent --no-window-system --eval \"addpath('"
        << script_dir << "'); " << function_name << "(" << args << ");\"";
    const int result = std::system(cmd.str().c_str());
    if (result != 0) {
        std::cerr << "[plot] Octave command failed (" << result << ").\n";
        return false;
    }
    return true;
}

bool runOctavePlot(const PlotOutputs& outputs, const std::string& script_path) {
    const std::filesystem::path script(script_path);
    const std::filesystem::path script_dir = script.parent_path();

    const std::string dir = script_dir.string();

    const bool ok_fused = runOctaveCommand(
        dir,
        "plot_trajectories",
        "'" + outputs.ground_truth_csv + "','" + outputs.fused_csv + "','" +
            outputs.ekf_csv + "','" + outputs.tag_csv + "','" +
            outputs.plot_png + "'");

    const bool ok_raw = runOctaveCommand(
        dir,
        "plot_trajectories_with_raw",
        "'" + outputs.ground_truth_csv + "','" + outputs.fused_csv + "','" +
            outputs.ekf_csv + "','" + outputs.wheel_only_csv + "','" +
            outputs.vision_csv + "','" + outputs.tag_csv + "','" +
            outputs.plot_with_raw_png + "'");

    const bool ok_yaw = runOctaveCommand(
        dir,
        "plot_yaw_headings",
        "'" + outputs.ground_truth_csv + "','" + outputs.fused_csv + "','" +
            outputs.ekf_csv + "','" + outputs.yaw_plot_png + "'");

    bool ok = ok_fused && ok_raw && ok_yaw;
    if (!ok_fused) {
        ok = runGnuplotPlot(outputs, PlotKind::FusedOnly) && ok;
    }
    if (!ok_raw) {
        ok = runGnuplotPlot(outputs, PlotKind::WithRaw) && ok;
    }
    if (!ok_yaw) {
        ok = runGnuplotPlot(outputs, PlotKind::Yaw) && ok;
    }
    return ok;
}
