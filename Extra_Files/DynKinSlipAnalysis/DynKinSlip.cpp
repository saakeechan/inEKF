/* ----------------------------------------------------------------------------
 * Dynamic Kinematic Slip Analysis
 * Analyzes base velocity estimates from FK foot velocities during contact regions
 * and compares against ground truth base velocity.
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <eigen3/Eigen/Dense>

// Include CSV parser and RobotModel
#include "../src/utility/csvParser.hpp"
#include "../src/utility/RobotModel.hpp"

/**
 * @brief Compute left foot velocity in world frame using numerical differentiation
 * @param data Parsed CSV data structure
 * @param lfoot_velocity_world Output: N x 3 matrix of left foot velocities
 * @param lfoot_velocity_norm Output: N x 1 vector of left foot velocity norms
 */
void compute_lfoot_velocity_world(const DataStruct& data, 
                                   std::vector<Eigen::Vector3d>& lfoot_velocity_world,
                                   std::vector<double>& lfoot_velocity_norm) {
    size_t N = data.num_rows;
    
    if (N < 2) {
        std::cerr << "Error: Need at least 2 data points" << std::endl;
        return;
    }
    
    lfoot_velocity_world.clear();
    lfoot_velocity_norm.clear();
    lfoot_velocity_world.reserve(N);
    lfoot_velocity_norm.reserve(N);
    
    // Compute numerical differentiation for left foot position
    for (size_t i = 0; i < N - 1; ++i) {
        Eigen::Vector3d pos_i(data.lfoot_pos_x[i], data.lfoot_pos_y[i], data.lfoot_pos_z[i]);
        Eigen::Vector3d pos_ip1(data.lfoot_pos_x[i+1], data.lfoot_pos_y[i+1], data.lfoot_pos_z[i+1]);
        
        double dt = data.timestamp[i+1] - data.timestamp[i];
        Eigen::Vector3d velocity = (pos_ip1 - pos_i) / dt;
        
        lfoot_velocity_world.push_back(velocity);
        lfoot_velocity_norm.push_back(velocity.norm() + 1e-8);
    }
    
    // Keep size consistent: duplicate the last velocity
    lfoot_velocity_world.push_back(lfoot_velocity_world.back());
    lfoot_velocity_norm.push_back(lfoot_velocity_norm.back());
}

// ---------------------------------------------------------------------------
// Contact region helpers
// ---------------------------------------------------------------------------
struct ContactRegion {
    size_t start;
    size_t end;   // inclusive
};

// Build rotation matrix from quaternion (w, x, y, z)
Eigen::Matrix3d quat_to_rot(double w, double x, double y, double z) {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q.toRotationMatrix();
}

// Skew-symmetric matrix (hat operator) from 3D vector
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return m;
}

// Extract contiguous contact regions from a binary contact signal
std::vector<ContactRegion> extract_contact_regions(
    const std::vector<double>& contact_signal)
{
    std::vector<ContactRegion> regions;
    size_t N = contact_signal.size();
    size_t i = 0;
    while (i < N) {
        if (contact_signal[i] > 0.5) {
            size_t start = i;
            while (i < N && contact_signal[i] > 0.5) ++i;
            size_t end = i - 1; // inclusive
            regions.push_back({start, end});
        } else {
            ++i;
        }
    }
    return regions;
}

struct RegionStats {
    double rmse_x, rmse_y, rmse_z, rmse_total;
    double mean_err_x, mean_err_y, mean_err_z;
    double sse_total;  // sum of squared errors for global RMSE
    size_t n_steps;
};

struct DetailedSample {
    double timestamp;
    Eigen::Vector3d foot_vel_world;
    Eigen::Vector3d foot_vel_from_body;  // foot velocity calculated from body frame
    Eigen::Vector3d error;
    size_t region_id;
};

// Analyse one contact region:
//   v_foot_world = v_base + R * v_foot_base + R * [omega]_× * p_foot_base
//   Zero-velocity constraint: v_foot_world = 0
//   => v_base_est = -R * v_foot_base - R * [omega]_× * p_foot_base
// Compare with ground truth v_base_world = [vel_x, vel_y, vel_z]
RegionStats analyse_region(
    const DataStruct& data,
    const ContactRegion& region,
    const std::vector<Eigen::Vector3d>& lfoot_velocity_world,
    std::vector<DetailedSample>* detailed_samples = nullptr,
    size_t region_id = 0,
    bool verbose = false)
{
    RegionStats stats{};
    stats.n_steps = region.end - region.start + 1;

    double se_x = 0, se_y = 0, se_z = 0, se_total = 0;
    double sum_ex = 0, sum_ey = 0, sum_ez = 0;

    for (size_t i = region.start; i <= region.end; ++i) {
        // Ground truth rotation matrix from quaternion (w, i, j, k)
        Eigen::Matrix3d R = quat_to_rot(data.quat_w[i], data.quat_i[i],
                                         data.quat_j[i], data.quat_k[i]);

        // Ground truth angular velocity (omega)
        Eigen::Vector3d omega(data.omega_x[i], data.omega_y[i], data.omega_z[i]);

        // FK left foot velocity in base frame (precomputed)
        Eigen::Vector3d v_foot_base(data.fk_left_foot_vel_x[i],
                                    data.fk_left_foot_vel_y[i],
                                    data.fk_left_foot_vel_z[i]);

        // FK left foot position in base frame (precomputed)
        Eigen::Vector3d p_foot_base(data.fk_left_foot_pos_x[i],
                                    data.fk_left_foot_pos_y[i],
                                    data.fk_left_foot_pos_z[i]);

        // Zero-velocity constraint with rotational moment compensation:

        // => v_base_est = -R * v_foot_base - [omega]_× * p_foot_base

        // Ground truth base velocity in world frame
        Eigen::Vector3d v_base_gt(data.vel_x[i], data.vel_y[i], data.vel_z[i]);
        
        Eigen::Vector3d r_world = R * p_foot_base;

        Eigen::Vector3d v_foot_world_from_body =
            v_base_gt
            + R * v_foot_base
            + omega.cross(r_world);

        Eigen::Vector3d v_base_est =
            -R * v_foot_base
            - omega.cross(r_world);
        

        Eigen::Vector3d err = v_base_est - v_base_gt;

        // Store detailed sample if requested
        if (detailed_samples) {
            DetailedSample sample;
            sample.timestamp = data.timestamp[i];
            sample.foot_vel_world = lfoot_velocity_world[i];
            sample.foot_vel_from_body = v_foot_world_from_body;
            sample.error = err;
            sample.region_id = region_id;
            detailed_samples->push_back(sample);
        }

        se_x     += err.x() * err.x();
        se_y     += err.y() * err.y();
        se_z     += err.z() * err.z();
        se_total += err.squaredNorm();
        sum_ex   += err.x();
        sum_ey   += err.y();
        sum_ez   += err.z();

        if (verbose) {
            std::printf("  t=%.4f  est=[%6.3f %6.3f %6.3f]  gt=[%6.3f %6.3f %6.3f]  err=[%6.3f %6.3f %6.3f]\n",
                data.timestamp[i],
                v_base_est.x(), v_base_est.y(), v_base_est.z(),
                v_base_gt.x(),  v_base_gt.y(),  v_base_gt.z(),
                err.x(), err.y(), err.z());
        }
    }

    double n = static_cast<double>(stats.n_steps);
    stats.rmse_x     = std::sqrt(se_x     / n);
    stats.rmse_y     = std::sqrt(se_y     / n);
    stats.rmse_z     = std::sqrt(se_z     / n);
    stats.rmse_total = std::sqrt(se_total / n);
    stats.sse_total  = se_total;  // store for global RMSE computation
    stats.mean_err_x = sum_ex / n;
    stats.mean_err_y = sum_ey / n;
    stats.mean_err_z = sum_ez / n;
    return stats;
}

int main(int argc, char** argv) {
    std::cout << "=== Dynamic Kinematic Slip Analysis ===\n" << std::endl;

    // Load CSV file
    std::string csv_path = (argc > 1) ? argv[1] : "../Data/robotstate_0_env2.csv";
    DataStruct data = ParseCSV(csv_path);
    std::cout << "Loaded " << data.num_rows << " rows from: " << csv_path << "\n" << std::endl;

    // Load robot model (used for name/config; FK already precomputed in CSV)
    std::string config_path = "../config/G1_config.yaml";
    RobotModel robot;
    if (!robot.load_config(config_path)) {
        std::cerr << "Error: Failed to load robot configuration!" << std::endl;
        return -1;
    }
    std::cout << "Robot model: " << robot.robot_name << "\n" << std::endl;

    // -------------------------------------------------------------------------
    // Compute left foot velocity in world frame
    // -------------------------------------------------------------------------
    std::vector<Eigen::Vector3d> lfoot_velocity_world;
    std::vector<double> lfoot_velocity_norm;
    compute_lfoot_velocity_world(data, lfoot_velocity_world, lfoot_velocity_norm);

    // -------------------------------------------------------------------------
    // 1. Extract left foot contact regions from ground truth contact signal
    // -------------------------------------------------------------------------
    auto lfoot_regions = extract_contact_regions(data.lfoot_contact);

    std::cout << "Left foot contact regions: " << lfoot_regions.size() << "\n" << std::endl;

    // -------------------------------------------------------------------------
    // 2. Analyse each contact region
    // -------------------------------------------------------------------------
    std::vector<DetailedSample> detailed_samples;
    double total_sse = 0.0;  // sum of squared errors (for global RMSE)
    size_t total_steps = 0;
    size_t region_idx = 0;

    std::printf("%-6s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n",
                "Reg#", "Start", "End", "Steps",
                "RMSE_x", "RMSE_y", "RMSE_z", "RMSE");
    std::printf("%s\n", std::string(74, '-').c_str());

    for (const auto& region : lfoot_regions) {
        RegionStats s = analyse_region(data, region, lfoot_velocity_world, 
                                        &detailed_samples, region_idx + 1, /*verbose=*/false);

        std::printf("%-6zu %-8zu %-8zu %-8zu "
                    "%-8.4f %-8.4f %-8.4f %-8.4f\n",
                    ++region_idx,
                    region.start, region.end, s.n_steps,
                    s.rmse_x, s.rmse_y, s.rmse_z, s.rmse_total);

        total_sse += s.sse_total;
        total_steps += s.n_steps;
    }

    // -------------------------------------------------------------------------
    // 3. Overall summary
    // -------------------------------------------------------------------------
    std::cout << "\n=== Overall Summary ===" << std::endl;
    std::printf("Total left foot contact steps analysed: %zu\n", total_steps);
    if (total_steps > 0) {
        double global_rmse = std::sqrt(total_sse / total_steps);
        std::printf("Global RMSE (3D):                       %.4f m/s\n", global_rmse);
    }

    // -------------------------------------------------------------------------
    // 4. Write detailed data to CSV for plotting
    // -------------------------------------------------------------------------
    // std::string output_csv = "../Extra_Files/DynKinSlipAnalysis/contact_analysis_output.csv";
    std::string output_csv = "contact_analysis_output.csv";
    
    std::ofstream outfile(output_csv);
    if (outfile.is_open()) {
        outfile << "timestamp,region_id,"
                << "foot_vel_x,foot_vel_y,foot_vel_z,foot_vel_norm,"
                << "foot_vel_body_x,foot_vel_body_y,foot_vel_body_z,foot_vel_body_norm\n";
        
        for (const auto& sample : detailed_samples) {
            double vel_norm = sample.foot_vel_world.norm();
            double vel_body_norm = sample.foot_vel_from_body.norm();
            outfile << std::fixed << std::setprecision(6)
                    << sample.timestamp << "," << sample.region_id << ","
                    << sample.foot_vel_world.x() << ","
                    << sample.foot_vel_world.y() << ","
                    << sample.foot_vel_world.z() << ","
                    << vel_norm << ","
                    << sample.foot_vel_from_body.x() << ","
                    << sample.foot_vel_from_body.y() << ","
                    << sample.foot_vel_from_body.z() << ","
                    << vel_body_norm << "\n";
        }
        outfile.close();
        std::cout << "\nDetailed analysis written to: " << output_csv << std::endl;
        std::cout << "Total samples: " << detailed_samples.size() << std::endl;
    } else {
        std::cerr << "Error: Could not write output CSV file" << std::endl;
    }

    // -------------------------------------------------------------------------
    // 5. Verbose detail for the first left foot contact region
    // -------------------------------------------------------------------------
    if (!lfoot_regions.empty()) {
        const auto& r = lfoot_regions[0];
        size_t show_end = std::min(r.start + 10, r.end);
        ContactRegion snippet{r.start, show_end};
        std::cout << "\n--- First left foot contact region (steps "
                  << r.start << " to " << show_end << ") ---" << std::endl;
        std::printf("  %-8s  %-22s  %-22s  %-22s\n",
                    "time", "est_vel (x y z)", "gt_vel  (x y z)", "error   (x y z)");
        analyse_region(data, snippet, lfoot_velocity_world, nullptr, 0, /*verbose=*/true);
    }

    return 0;
}
 


