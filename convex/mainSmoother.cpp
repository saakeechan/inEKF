/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------
 * LeggedEstimatorReplayExample
 *
 * Suggested workflow:
 * 1. Build this example from the build directory:
 *      make -j6 LeggedEstimatorReplayExample
 *
 * 2. Run the replay on the staircase example dataset:
 *      ./examples/LeggedEstimatorReplayExample \
 *          --output /tmp/.../outputs
 *
 *    The example defaults to `examples/Data/legged_staircase/` and runs all
 *    four C++ variants. For shorter runs, optionally restrict the replay to an
 *    initial time window or choose a subset of variants:
 *      ./examples/LeggedEstimatorReplayExample \
 *          --output /tmp/.../outputs \
 *          --max-duration-seconds 8.0 \
 *          --variants invariant_ekf,invariant_graph,fixed_lag_single_bias
 *
 * 3. Visualize the saved trajectories and metrics in:
 *      gtsam/navigation/doc/LeggedEstimator.ipynb
 *
 * The runner writes:
 *   - <variant>_trajectory.csv : one trajectory per estimator variant
 *   - <variant>_metrics.csv    : one metrics row per estimator variant
 *   - contact_packet_usage.csv : available vs used contact packets for all feet
 * -------------------------------------------------------------------------- */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/dataset.h>

#include "LeggedEstimatorReplayUtils.h"
#include "src/utility/csvParser.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

//
// FOOT VELOCITY TRACKING SYSTEM:
// This implementation tracks THREE types of foot velocities for comparison:
//   1. Ground Truth Velocities: Finite differencing of GT foot positions from CSV
//      - Uses lfoot_pos_x/y/z and rfoot_pos_x/y/z from G1LoggingData.csv
//   2. Estimated Velocities: Finite differencing of factor graph foothold estimates
//      - Computes velocity from estimated foothold positions in world frame
//   3. Forward Kinematics Velocities: Direct FK velocities from CSV
//      - Uses fk_left_foot_vel_x/y/z and fk_right_foot_vel_x/y/z from CSV
// All velocities are kept separate for left and right legs
// Output CSV format: timestamp, gt_vx/vy/vz, est_vx/vy/vz, fk_vx/vy/vz, contact_state
// ------------------------------------------------------------




namespace gtsam {
namespace {

namespace fs = std::filesystem;

struct ReplayConfig : public LeggedEstimatorParams {
  Vector3 gravity = Vector3(0.0, 0.0, -9.81);
  Point3 initialPosition = Point3(0.0, 0.0, 0.0);  // [Px, Py, Pz] from config
  Vector3 initialVelocity = Vector3::Zero();  // [Vx, Vy, Vz] from config
  Vector initialBaseCovarianceDiagonal =
      (Vector(9) << 1e-10, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10)
          .finished();
  double sigmaGyro = std::sqrt(1e-2);
  double sigmaIntegration = 1e-3;
  double sigmaAcc = std::sqrt(1e1);
  double biasAccRandomWalkSigma = std::sqrt(1e-10);
  double biasOmegaRandomWalkSigma = std::sqrt(1e-10);
  double lagSeconds = 10;
  double maxDeadReckoningSeconds = 0.01;
  
  // Constructor to explicitly initialize all LeggedEstimatorParams
  ReplayConfig() {
    // Body-IMU transform: identity (no rotation, zero offset)
    body_P_imu = Pose3(Rot3(), Point3(0.0, 0.0, 0.0));
    
    // Contact covariance (contact: -5 → variance = 10^-5)
    const double contactVariance = std::sqrt(1e-6);
    contactCovariance = Vector3(contactVariance, contactVariance, contactVariance * 1e-0).asDiagonal();
    
    // Foothold parameters (matching GTSAM example defaults - already correct from base class)
    footholdInitSigma = 1e-6;       // Initial foothold uncertainty (m)
    footholdProcessSigma = 1e-6;   // Foothold process noise (m)

    // Full contact initialization (set via command line flag, but default here)
    useFullContactInitialization = false;
    heightPriorSigma = 0.15;       // Robot height prior uncertainty (m) // Doesnt matter cauae useFullContactInitialization=false
    
    // Robust noise and marginalization (IMPORTANT: these differ from GTSAM base class defaults)
    useRobustContactNoise = false;   // Set to true to use Huber loss
    robustContactHuberK = 2;       // Huber loss threshold
    marginalizeLeavingFoot = true;   // Marginalize footholds when feet leave ground

  }
};

// Structure to hold ground truth initial state from CSV
struct GroundTruthInitialState {
  Point3 position;
  Rot3 orientation;
  Vector3 velocity;
  
  GroundTruthInitialState() 
    : position(0.0, 0.0, 0.0), 
      orientation(Rot3()), 
      velocity(Vector3::Zero()) {}
};

// Structure to hold both ground truth initial state and full CSV data
struct GroundTruthData {
  GroundTruthInitialState initialState;
  DataStruct fullData;
  bool dataLoaded = false;
};

// Load ground truth data from CSV file (used for both initialization and foot velocity calculations)
GroundTruthData loadGroundTruthData(const fs::path& datasetDir) {
  GroundTruthData gtData;
  
  fs::path gtPath = datasetDir / "robotstate_0_800.csv";
  if (!fs::exists(gtPath)) {
    std::cout << "Warning: Ground truth file not found at " << gtPath.string() 
              << ", using default initial state\n";
    return gtData;
  }
  
  // Parse the ground truth CSV
  DataStruct csvData = ParseCSV(gtPath.string());
  
  if (csvData.num_rows == 0) {
    std::cout << "Warning: Ground truth CSV is empty, using default initial state\n";
    return gtData;
  }
  
  // Store full CSV data for foot velocity calculations
  gtData.fullData = csvData;
  gtData.dataLoaded = true;
  
  // Extract first timestep data (index 0) for initialization
  gtData.initialState.position = Point3(csvData.pos_x[0], csvData.pos_y[0], csvData.pos_z[0]);
  gtData.initialState.velocity = Vector3(csvData.vel_x[0], csvData.vel_y[0], csvData.vel_z[0]);
  
  // Convert quaternion to Rot3
  // DataStruct has: quat_w, quat_i, quat_j, quat_k
  Eigen::Quaterniond quat(csvData.quat_w[0], csvData.quat_i[0], 
                          csvData.quat_j[0], csvData.quat_k[0]);
  gtData.initialState.orientation = Rot3(quat);
  
  std::cout << "Loaded ground truth data from " << gtPath.string() << " (" << csvData.num_rows << " rows):\n";
  std::cout << "  Initial Position: [" << gtData.initialState.position.x() << ", " 
            << gtData.initialState.position.y() << ", " << gtData.initialState.position.z() << "]\n";
  std::cout << "  Initial Velocity: [" << gtData.initialState.velocity(0) << ", " 
            << gtData.initialState.velocity(1) << ", " << gtData.initialState.velocity(2) << "]\n";
  std::cout << "  Initial Orientation (RPY): Roll=" << csvData.roll[0] << " rad, Pitch=" 
            << csvData.pitch[0] << " rad, Yaw=" << csvData.yaw[0] << " rad\n";
  
  return gtData;
}

// 

enum class ContactPacketStatus {
  kIgnored,
  kUsedTouchdown,
  kUsedPeriodic,
};

struct ContactPacketDecision {
  size_t eventIndex = 0;
  double timestampS = 0.0;
  std::vector<ContactMeasurement> activeContacts;
  ContactPacketStatus status = ContactPacketStatus::kIgnored;
};

struct ContactReplayPlan {
  std::vector<ContactPacketDecision> packetDecisions;
  std::vector<size_t> scheduledPacketIndices;
};

std::optional<ContactEvent> firstFullContactEvent(const Dataset& dataset) {
  for (const ContactEvent& event : dataset.contactEvents) {
    if (event.activeContacts.size() == dataset.metadata.footNames.size()) {
      return event;
    }
  }
  return std::nullopt;
}

struct InitialBiasEstimate {
  imuBias::ConstantBias bias;
  std::string source;
  size_t sampleCount = 0;
};

// -----------------------Initialization Helpers-----------------------


// Convert contact meaurements from body frame to IMU frame and compute an initial attitude from the contact plane normal.
NavState fullContactInitializationNavState(
    const ReplayConfig& replayConfig,
    const std::vector<ContactMeasurement>& activeContacts) {
  Matrix imuContacts(3, static_cast<Eigen::Index>(activeContacts.size()));
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    imuContacts.col(static_cast<Eigen::Index>(index)) =
        replayConfig.body_P_imu.transformTo(
            Point3(activeContacts[index].bodyPoint));
  }


  const Vector3 centroid = imuContacts.rowwise().mean();
  const Matrix centered = imuContacts.colwise() - centroid;
  Vector3 normal =
      Eigen::JacobiSVD<Matrix>(centered, Eigen::ComputeFullU).matrixU().col(2);
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  const double roll = std::atan2(normal.y(), normal.z());
  const double pitch = std::asin(-normal.x());
  const Rot3 attitude = Rot3::Ypr(0.0, pitch, roll);

  double height = 0.0;
  for (const ContactMeasurement& contact : activeContacts) {
    const Point3 measurement =
        replayConfig.body_P_imu.transformTo(Point3(contact.bodyPoint));
    height -= attitude.matrix().row(2).dot(measurement);
  }
  height /= static_cast<double>(activeContacts.size());
  return NavState(attitude, Point3(0.0, 0.0, height), Vector3::Zero());
}

double initialBiasWindowEndTime(const Dataset& dataset, double startTime) {
  double endTime = startTime + 1.0;
  bool seenStart = false;
  for (const ContactEvent& event : dataset.contactEvents) {
    if (!seenStart) {
      if (std::abs(event.timestampS - startTime) < 1e-12) {
        seenStart = true;
      }
      continue;
    }
    if (event.activeContacts.size() < dataset.metadata.footNames.size()) {
      endTime = std::min(endTime, event.timestampS);
      break;
    }
  }
  return endTime;
}

bool samplesLookStationary(const std::vector<ImuSample>& samples,
                           double gravityMagnitude) {
  if (samples.empty()) {
    return false;
  }
  double meanOmegaNorm = 0.0;
  double meanAccelNorm = 0.0;
  for (const ImuSample& sample : samples) {
    meanOmegaNorm += sample.omega.norm();
    meanAccelNorm += sample.specificForce.norm();
  }
  meanOmegaNorm /= static_cast<double>(samples.size());
  meanAccelNorm /= static_cast<double>(samples.size());
  return meanOmegaNorm < 1e-2 &&
         std::abs(meanAccelNorm - gravityMagnitude) < 0.15;
}

std::vector<ImuSample> initialStaticImuSamplesBefore(const Dataset& dataset,
                                                     double endTime) {
  std::vector<ImuSample> samples;
  const double startTime = dataset.imuSamples.front().timestampS;
  const double gravityMagnitude = ReplayConfig().gravity.norm();
  constexpr double kMaxWindowSeconds = 4.0;
  constexpr double kMaxOmegaNorm = 1e-2;
  constexpr double kMaxAccelNormError = 0.2;

  for (const ImuSample& sample : dataset.imuSamples) {
    if (sample.timestampS > endTime ||
        sample.timestampS - startTime > kMaxWindowSeconds) {
      break;
    }
    const bool stationary = sample.omega.norm() <= kMaxOmegaNorm &&
                            std::abs(sample.specificForce.norm() -
                                     gravityMagnitude) <= kMaxAccelNormError;
    if (!stationary && !samples.empty()) {
      break;
    }
    if (stationary) {
      samples.push_back(sample);
    }
  }

  if (samples.empty()) {
    const size_t fallbackCount =
        std::min<size_t>(dataset.imuSamples.size(), 200);
    samples.assign(dataset.imuSamples.begin(),
                   dataset.imuSamples.begin() + fallbackCount);
  }
  return samples;
}

std::vector<ImuSample> initialStationaryImuSamples(const Dataset& dataset,
                                                   double startTime,
                                                   double endTime) {
  std::vector<ImuSample> samples;

  for (const ImuSample& sample : dataset.imuSamples) {
    if (sample.timestampS < startTime) {
      continue;
    }
    if (sample.timestampS > endTime) {
      break;
    }
    samples.push_back(sample);
  }

  if (samples.empty()) {
    const size_t fallbackCount =
        std::min<size_t>(dataset.imuSamples.size(), 200);
    samples.assign(dataset.imuSamples.begin(),
                   dataset.imuSamples.begin() + fallbackCount);
  }
  return samples;
}

imuBias::ConstantBias estimateBiasFromSamples(
    const std::vector<ImuSample>& samples, const NavState& initialNavState,
    const ReplayConfig& replayConfig) {
  Vector3 meanOmega = Vector3::Zero();
  Vector3 meanSpecificForce = Vector3::Zero();
  for (const ImuSample& sample : samples) {
    meanOmega += sample.omega;
    meanSpecificForce += sample.specificForce;
  }
  meanOmega /= static_cast<double>(samples.size());
  meanSpecificForce /= static_cast<double>(samples.size());

  const Vector3 expectedSpecificForce =
      initialNavState.attitude().unrotate(-replayConfig.gravity);
  const Vector3 accelBias = meanSpecificForce - expectedSpecificForce;
  return imuBias::ConstantBias(accelBias, meanOmega);
}

InitialBiasEstimate estimateInitialImuBias(const Dataset& dataset) {
  const ReplayConfig replayConfig;
  const std::optional<ContactEvent> event = firstFullContactEvent(dataset);
  if (!event) {
    return {imuBias::ConstantBias(), "none", 0};
  }
  const double endTime = initialBiasWindowEndTime(dataset, event->timestampS);
  const std::vector<ImuSample> postInitSamples =
      initialStationaryImuSamples(dataset, event->timestampS, endTime);
  const NavState initialNavState =
      fullContactInitializationNavState(replayConfig, event->activeContacts);
  if (samplesLookStationary(postInitSamples, replayConfig.gravity.norm())) {
    return {
        estimateBiasFromSamples(postInitSamples, initialNavState, replayConfig),
        "post-init full-contact", postInitSamples.size()};
  }

  const std::vector<ImuSample> preInitSamples =
      initialStaticImuSamplesBefore(dataset, event->timestampS);
  return {
      estimateBiasFromSamples(preInitSamples, initialNavState, replayConfig),
      "pre-init static fallback", preInitSamples.size()};
}




// Look at these intialization methods later because we might just have 
// the option to initialize with known values 
// -------------------------------Initialization Helpers-----------------------


LeggedEstimatorParams makeParams(const ReplayConfig& replayConfig,
                                 const bool disableFullContactInitialization,
                                 const imuBias::ConstantBias& imuBiasEstimate) {
  auto preintegrationParams =
      std::make_shared<PreintegrationParams>(replayConfig.gravity);
  preintegrationParams->gyroscopeCovariance =
      Matrix3::Identity() * (replayConfig.sigmaGyro * replayConfig.sigmaGyro);
  preintegrationParams->integrationCovariance =
      Matrix3::Identity() *
      (replayConfig.sigmaIntegration * replayConfig.sigmaIntegration);
  preintegrationParams->accelerometerCovariance =
      Matrix3::Identity() * (replayConfig.sigmaAcc * replayConfig.sigmaAcc);

  LeggedEstimatorParams params = replayConfig;
  params.preintegrationParams = preintegrationParams;
  params.imuBias = imuBiasEstimate;
  params.useFullContactInitialization = !disableFullContactInitialization;
  return params;
}

NavState makeInitialState(const ReplayConfig& replayConfig, 
                          const GroundTruthInitialState& gtInitialState) {
  // Use ground truth if available, otherwise use default from config
  const Point3& position = gtInitialState.position.norm() > 0 ? gtInitialState.position : replayConfig.initialPosition;
  const Vector3& velocity = gtInitialState.velocity.norm() > 0 ? gtInitialState.velocity : replayConfig.initialVelocity;
  const Rot3& orientation = gtInitialState.orientation;
  
  return NavState(orientation, position, velocity);
}

Matrix makeInitialCovariance(size_t numFeet, const ReplayConfig& replayConfig) {
  const int dim = 9 + 3 * static_cast<int>(numFeet);
  Matrix covariance = Matrix::Zero(dim, dim);
  covariance.diagonal().head(9) = replayConfig.initialBaseCovarianceDiagonal;
  const double footholdVariance =
      replayConfig.footholdInitSigma * replayConfig.footholdInitSigma;
  for (size_t foot = 0; foot < numFeet; ++foot) {
    covariance.diagonal()
        .segment(9 + 3 * static_cast<int>(foot), 3)
        .setConstant(footholdVariance);
  }
  return covariance;
}

Matrix9 makeInitialBaseCovariance(const ReplayConfig& replayConfig) {
  Matrix9 covariance = Matrix9::Zero();
  covariance.diagonal() = replayConfig.initialBaseCovarianceDiagonal;
  return covariance;
}

Matrix makeInitialFootholds(size_t numFeet) {
  return Matrix::Zero(3, static_cast<Eigen::Index>(numFeet));
}

NavState navStateFromEstimate(const ExtendedPose3d& estimate) {
  return NavState(estimate.rotation(), estimate.x(0), estimate.x(1));
}

Matrix footholdsFromEstimate(const ExtendedPose3d& estimate) {
  const Eigen::Index numFeet =
      static_cast<Eigen::Index>(estimate.k() - static_cast<size_t>(2));
  return estimate.xMatrix().rightCols(numFeet);
}

double contactResidualNorm(const ExtendedPose3d& estimate,
                           const Pose3& bodyPImu,
                           const ContactMeasurement& contact) {
  const Matrix footholds = footholdsFromEstimate(estimate);
  const NavState navState = navStateFromEstimate(estimate);
  const Point3 foothold(footholds.col(static_cast<Eigen::Index>(contact.foot)));
  const Point3 prediction = navState.pose().transformTo(foothold);
  const Point3 measurement = bodyPImu.transformTo(Point3(contact.bodyPoint));
  return (prediction - measurement).norm();
}

bool hasTouchdown(const std::vector<ContactMeasurement>& contacts) {
  return std::any_of(
      contacts.begin(), contacts.end(),
      [](const ContactMeasurement& contact) { return contact.touchdown; });
}

const char* contactRowStatusName(ContactPacketStatus packetStatus,
                                 bool touchdown) {
  if (touchdown) {
    return "touchdown";
  }
  switch (packetStatus) {
    case ContactPacketStatus::kIgnored:
      return "ignored";
    case ContactPacketStatus::kUsedTouchdown:
      return "touchdown_support_used";
    case ContactPacketStatus::kUsedPeriodic:
      return "periodic_used";
  }
  return "unknown";
}

const char* contactRowColor(ContactPacketStatus packetStatus, bool touchdown) {
  if (touchdown) {
    return "red";
  }
  switch (packetStatus) {
    case ContactPacketStatus::kIgnored:
      return "green";
    case ContactPacketStatus::kUsedTouchdown:
    case ContactPacketStatus::kUsedPeriodic:
      return "blue";
  }
  return "black";
}

ContactReplayPlan buildContactReplayPlan(const Dataset& dataset,
                                         double maxDeadReckoningSeconds) {
  ContactReplayPlan plan;
  plan.packetDecisions.reserve(dataset.contactEvents.size());

  double lastUpdateTime = -std::numeric_limits<double>::infinity();
  for (const ContactEvent& event : dataset.contactEvents) {
    ContactPacketDecision decision;
    decision.eventIndex = event.index;
    decision.timestampS = event.timestampS;
    decision.activeContacts = event.activeContacts;

    if (hasTouchdown(event.activeContacts)) {
      decision.status = ContactPacketStatus::kUsedTouchdown;
    } else if (maxDeadReckoningSeconds <= 0.0 ||
               (std::isfinite(lastUpdateTime) &&
                event.timestampS - lastUpdateTime >=
                    maxDeadReckoningSeconds - 1e-12)) {
      decision.status = ContactPacketStatus::kUsedPeriodic;
    } else {
      decision.status = ContactPacketStatus::kIgnored;
    }

    if (decision.status != ContactPacketStatus::kIgnored) {
      plan.scheduledPacketIndices.push_back(plan.packetDecisions.size());
      lastUpdateTime = event.timestampS;
    }

    plan.packetDecisions.push_back(std::move(decision));
  }

  return plan;
}

std::unique_ptr<LeggedEstimator> makeEstimator(
    const std::string& filterName, const std::vector<std::string>& footNames,
    const NavState& initialState, const Matrix& footholds,
    const Matrix& covariance, const Matrix9& baseCovariance,
    const LeggedEstimatorParams& params, double lagSeconds) {
  if (filterName == "invariant_ekf") {
    return std::make_unique<LeggedInvariantEKF>(initialState, footholds,
                                                covariance, params, footNames);
  }
  if (filterName == "invariant_graph") {
    return std::make_unique<LeggedInvariantIEKF>(initialState, footholds,
                                                 covariance, params, footNames);
  }
  if (filterName == "fixed_lag_single_bias") {
    return std::make_unique<LeggedFixedLagSmoother>(
        initialState, footholds, baseCovariance, params, lagSeconds, footNames);
  }
  if (filterName == "fixed_lag_combined_bias") {
    return std::make_unique<LeggedCombinedFixedLagSmoother>(
        initialState, footholds, baseCovariance, params, lagSeconds, footNames);
  }
  throw std::runtime_error("Unknown filter name: " + filterName);
}

ReplayOutputs replayFilter(const std::string& filterName,
                           const Dataset& dataset,
                           const ContactReplayPlan& contactReplayPlan,
                           const ReplayConfig& replayConfig,
                           std::ofstream& trajectoryOutput,
                           std::ofstream& footVelocityOutput,
                           const double maxDurationSeconds,
                           const bool disableFullContactInitialization,
                           const imuBias::ConstantBias& imuBiasEstimate,
                           const GroundTruthInitialState& gtInitialState,
                           const DataStruct& gtFootData,
                           const bool haveGroundTruthData) {
  const NavState initialState = makeInitialState(replayConfig, gtInitialState);
  const Matrix footholds =
      makeInitialFootholds(dataset.metadata.footNames.size());
  const Matrix covariance =
      makeInitialCovariance(dataset.metadata.footNames.size(), replayConfig);
  const Matrix9 baseCovariance = makeInitialBaseCovariance(replayConfig);
  const LeggedEstimatorParams params = makeParams(
      replayConfig, disableFullContactInitialization, imuBiasEstimate);
  std::unique_ptr<LeggedEstimator> estimator = makeEstimator(
      filterName, dataset.metadata.footNames, initialState, footholds,
      covariance, baseCovariance, params, replayConfig.lagSeconds);

  // ========================================================================
  // PRINT ALL INITIALIZATION PARAMETERS FOR VERIFICATION
  // ========================================================================
  std::cout << "\n========== INITIALIZATION PARAMETERS ==========\n";
  std::cout << "Filter Name: " << filterName << "\n\n";
  
  std::cout << "--- ReplayConfig ---\n";
  std::cout << "Gravity: [" << replayConfig.gravity.transpose() << "]\n";
  std::cout << "Initial Position: [" << replayConfig.initialPosition.transpose() << "]\n";
  std::cout << "Initial Velocity: [" << replayConfig.initialVelocity.transpose() << "]\n";
  std::cout << "Sigma Gyro: " << replayConfig.sigmaGyro << "\n";
  std::cout << "Sigma Integration: " << replayConfig.sigmaIntegration << "\n";
  std::cout << "Sigma Acc: " << replayConfig.sigmaAcc << "\n";
  std::cout << "Bias Acc Random Walk Sigma: " << replayConfig.biasAccRandomWalkSigma << "\n";
  std::cout << "Bias Omega Random Walk Sigma: " << replayConfig.biasOmegaRandomWalkSigma << "\n";
  std::cout << "Lag Seconds: " << replayConfig.lagSeconds << "\n";
  std::cout << "Max Dead Reckoning Seconds: " << replayConfig.maxDeadReckoningSeconds << "\n\n";
  
  std::cout << "--- Body-IMU Transform (body_P_imu) ---\n";
  std::cout << replayConfig.body_P_imu.matrix() << "\n\n";
  
  std::cout << "--- Initial State (from Ground Truth) ---\n";
  std::cout << "Position: [" << initialState.position().transpose() << "]\n";
  std::cout << "Velocity: [" << initialState.velocity().transpose() << "]\n";
  std::cout << "Attitude (RPY): [" << initialState.attitude().rpy().transpose() << "]\n\n";
  
  std::cout << "--- IMU Bias Estimate ---\n";
  std::cout << "Accelerometer Bias: [" << imuBiasEstimate.accelerometer().transpose() << "]\n";
  std::cout << "Gyroscope Bias: [" << imuBiasEstimate.gyroscope().transpose() << "]\n";
  std::cout << "Bias Source: " << "initial estimate" << "\n\n";
  
  std::cout << "--- LeggedEstimatorParams (from params) ---\n";
  std::cout << "Use Full Contact Initialization: " << (params.useFullContactInitialization ? "true" : "false") << "\n";
  std::cout << "Disable Full Contact Init Flag: " << (disableFullContactInitialization ? "true" : "false") << "\n";
  std::cout << "Foothold Init Sigma: " << params.footholdInitSigma << " m (explicitly set in ReplayConfig)\n";
  std::cout << "Foothold Process Sigma: " << params.footholdProcessSigma << " m (explicitly set in ReplayConfig)\n";
  std::cout << "Height Prior Sigma: " << params.heightPriorSigma << " m (explicitly set in ReplayConfig)\n";
  std::cout << "Use Robust Contact Noise: " << (params.useRobustContactNoise ? "true" : "false") << " (explicitly set in ReplayConfig)\n";
  std::cout << "Robust Contact Huber K: " << params.robustContactHuberK << " (explicitly set in ReplayConfig)\n";
  std::cout << "Marginalize Leaving Foot: " << (params.marginalizeLeavingFoot ? "true" : "false") << " (explicitly set in ReplayConfig)\n";
  std::cout << "Bias Acc Random Walk Sigma (in params): " << params.biasAccRandomWalkSigma << " (from ReplayConfig)\n";
  std::cout << "Bias Omega Random Walk Sigma (in params): " << params.biasOmegaRandomWalkSigma << " (from ReplayConfig)\n\n";
  
  std::cout << "--- Contact Covariance ---\n";
  std::cout << params.contactCovariance.diagonal().transpose() << "\n\n";
  
  std::cout << "--- Preintegration Params ---\n";
  if (params.preintegrationParams) {
    std::cout << "Gyroscope Covariance diagonal: " << params.preintegrationParams->gyroscopeCovariance.diagonal().transpose() << "\n";
    std::cout << "Accelerometer Covariance diagonal: " << params.preintegrationParams->accelerometerCovariance.diagonal().transpose() << "\n";
    std::cout << "Integration Covariance diagonal: " << params.preintegrationParams->integrationCovariance.diagonal().transpose() << "\n";
  }
  
  std::cout << "--- Initial Base Covariance Diagonal ---\n";
  std::cout << replayConfig.initialBaseCovarianceDiagonal.transpose() << "\n";
  
  std::cout << "--- Dataset Info ---\n";
  std::cout << "Number of Feet: " << dataset.metadata.footNames.size() << "\n";
  std::cout << "Foot Names: ";
  for (size_t i = 0; i < dataset.metadata.footNames.size(); ++i) {
    std::cout << dataset.metadata.footNames[i];
    if (i < dataset.metadata.footNames.size() - 1) std::cout << ", ";
  }
  std::cout << "\n";
  std::cout << "Total IMU Samples: " << dataset.imuSamples.size() << "\n";
  std::cout << "Total Contact Events: " << dataset.contactEvents.size() << "\n";
  std::cout << "Max Duration: " << maxDurationSeconds << " s\n";
  std::cout << "===============================================\n\n";

  ReplayMetrics metrics;
  const Pose3& bodyPImu = replayConfig.body_P_imu;
  const double startTimestamp = dataset.imuSamples.front().timestampS;
  NavState previousLoggedState = navStateFromEstimate(estimator->estimate());
  bool havePreviousLoggedState = false;

  // ========================================================================
  // FOOT VELOCITY TRACKING STRUCTURES (using preloaded ground truth data)
  // ========================================================================
  const bool haveGroundTruthFootData = haveGroundTruthData;
  const size_t numFeet = dataset.metadata.footNames.size();
  
  // FK world foot position tracking (FK body-frame positions transformed into world frame)
  std::vector<Vector3> previousFKFootPositionsWorld(numFeet, Vector3::Zero());
  double previousFKFootWorldTime = 0.0;
  bool havePreviousFKFootPositionsWorld = false;

  // Ground truth foot position tracking (from CSV)
  std::vector<Vector3> previousGTFootPositions(numFeet, Vector3::Zero());
  double previousGTFootTime = 0.0;
  bool havePreviousGTFootPositions = false;

  // CSV time index tracking (GT positions are constant while the index doesn't change)
  size_t previousCsvTimeIndex = 0;
  bool havePreviousCsvTimeIndex = false;
  
  // Helper lambda to extract GT foot positions from CSV at given time index
  auto getGTFootPositions = [&](size_t timeIndex) -> std::vector<Vector3> {
    std::vector<Vector3> positions(numFeet);
    if (haveGroundTruthFootData && timeIndex < gtFootData.num_rows) {
      // Assuming 2 feet: left=0, right=1
      if (numFeet >= 2) {
        positions[0] = Vector3(gtFootData.lfoot_pos_x[timeIndex], 
                               gtFootData.lfoot_pos_y[timeIndex], 
                               gtFootData.lfoot_pos_z[timeIndex]);
        positions[1] = Vector3(gtFootData.rfoot_pos_x[timeIndex], 
                               gtFootData.rfoot_pos_y[timeIndex], 
                               gtFootData.rfoot_pos_z[timeIndex]);
      }
    }
    return positions;
  };
  
  // Helper lambda to extract FK foot velocities from CSV at given time index
  // FK FORMULA: v_foot_world = v_body + R * v_foot_body + R * [omega - bias_gyro]× * p_foot_body
  // Where:
  //   v_body = body velocity in world frame (from ESTIMATED STATE)
  //   R = rotation matrix from body to world frame (from ESTIMATED STATE)
  //   v_foot_body = foot velocity in body frame from FK joints (from CSV: fk_*_foot_vel_*)
  //   omega = angular velocity (from CSV: gyro_body_x/y/z or omega_x/y/z)
  //   bias_gyro = gyroscope bias (from ESTIMATED STATE)
  //   p_foot_body = foot position in body frame (from CSV: fk_*_foot_pos_*)
  //   [.]× = skew-symmetric matrix (cross product operator)
  auto getFKFootVelocities = [&](size_t timeIndex, const Vector3& v_body_est, 
                                  const Matrix3& R_est, const Vector3& bias_gyro_est) -> std::vector<Vector3> {
    std::vector<Vector3> velocities(numFeet, Vector3::Zero());
    if (haveGroundTruthFootData && timeIndex < gtFootData.num_rows) {
      // Use ESTIMATED body velocity (world frame) from state
      const Vector3& v_body = v_body_est;
      
      // Use ESTIMATED rotation matrix from state
      const Matrix3& R = R_est;
      
      // Extract angular velocity (body frame) from CSV
      Vector3 omega(gtFootData.gyro_body_x[timeIndex], 
                    gtFootData.gyro_body_y[timeIndex], 
                    gtFootData.gyro_body_z[timeIndex]);
      
      // Use ESTIMATED gyroscope bias from state
      Vector3 omega_unbiased = omega - bias_gyro_est;
      
      // Assuming 2 feet: left=0, right=1
      if (numFeet >= 2) {
        // LEFT FOOT FK velocity calculation
        Vector3 v_left_body(gtFootData.fk_left_foot_vel_x[timeIndex], 
                            gtFootData.fk_left_foot_vel_y[timeIndex], 
                            gtFootData.fk_left_foot_vel_z[timeIndex]);
        Vector3 p_left_body(gtFootData.fk_left_foot_pos_x[timeIndex], 
                            gtFootData.fk_left_foot_pos_y[timeIndex], 
                            gtFootData.fk_left_foot_pos_z[timeIndex]);
        // v_foot_world = v_body + R * (v_foot_body + omega × p_foot_body)
        velocities[0] = v_body + R * v_left_body + R * omega_unbiased.cross(p_left_body);
        
        // RIGHT FOOT FK velocity calculation
        Vector3 v_right_body(gtFootData.fk_right_foot_vel_x[timeIndex], 
                             gtFootData.fk_right_foot_vel_y[timeIndex], 
                             gtFootData.fk_right_foot_vel_z[timeIndex]);
        Vector3 p_right_body(gtFootData.fk_right_foot_pos_x[timeIndex], 
                             gtFootData.fk_right_foot_pos_y[timeIndex], 
                             gtFootData.fk_right_foot_pos_z[timeIndex]);
        velocities[1] = v_body + R * v_right_body + R * omega_unbiased.cross(p_right_body);
      }
    }
    return velocities;
  };

  // Helper lambda to compute FK foot positions in world frame.
  // p_foot_world = pose.transformFrom(p_foot_body)
  auto getFKFootPositionsWorld = [&](size_t timeIndex, const Pose3& pose) -> std::vector<Vector3> {
    std::vector<Vector3> positions(numFeet, Vector3::Zero());
    if (haveGroundTruthFootData && timeIndex < gtFootData.num_rows) {
      if (numFeet >= 2) {
        const Vector3 p_left_body(gtFootData.fk_left_foot_pos_x[timeIndex],
                                  gtFootData.fk_left_foot_pos_y[timeIndex],
                                  gtFootData.fk_left_foot_pos_z[timeIndex]);
        positions[0] = pose.transformFrom(p_left_body);

        const Vector3 p_right_body(gtFootData.fk_right_foot_pos_x[timeIndex],
                                   gtFootData.fk_right_foot_pos_y[timeIndex],
                                   gtFootData.fk_right_foot_pos_z[timeIndex]);
        positions[1] = pose.transformFrom(p_right_body);
      }
    }
    return positions;
  };

  // Helper lambda to get contact states from CSV
  auto getContactStates = [&](size_t timeIndex) -> std::vector<bool> {
    std::vector<bool> contacts(numFeet, false);
    if (haveGroundTruthFootData && timeIndex < gtFootData.num_rows) {
      if (numFeet >= 2) {
        contacts[0] = (gtFootData.lfoot_contact[timeIndex] > 0.5);
        contacts[1] = (gtFootData.rfoot_contact[timeIndex] > 0.5);
      }
    }
    return contacts;
  };
  
  // Helper lambda to find closest CSV time index for a given timestamp.
  // Uses binary search (O(log N)) since CSV timestamps are sorted ascending.
  auto findClosestTimeIndex = [&](double timestampS) -> size_t {
    if (!haveGroundTruthFootData || gtFootData.num_rows == 0) return 0;

    const auto& ts = gtFootData.timestamp;
    const auto it = std::lower_bound(ts.begin(), ts.end(), timestampS);
    if (it == ts.end()) return gtFootData.num_rows - 1;
    if (it == ts.begin()) return 0;
    const auto prev = std::prev(it);
    // Pick whichever neighbour is closer
    return (timestampS - *prev <= *it - timestampS)
        ? static_cast<size_t>(std::distance(ts.begin(), prev))
        : static_cast<size_t>(std::distance(ts.begin(), it));
  };

  auto logState = [&](const std::string& kind, double timestampS) {
    const ExtendedPose3d estimate = estimator->estimate();
    const NavState state = navStateFromEstimate(estimate);
    writeTrajectoryRow(trajectoryOutput, kind, timestampS, state);
    ++metrics.trajectoryRows;
    if (havePreviousLoggedState) {
      metrics.pathLength +=
          (state.position() - previousLoggedState.position()).norm();
    }
    previousLoggedState = state;
    havePreviousLoggedState = true;
  };

  auto startWallTime = std::chrono::steady_clock::now();
  logState("start", startTimestamp);

  size_t imuIndex = 0;
  size_t updateIndex = 0;
  ImuSample heldImu = dataset.imuSamples.front();
  double currentTime = heldImu.timestampS;

  while (true) {
    const double nextImuTime = (imuIndex + 1 < dataset.imuSamples.size())
                                   ? dataset.imuSamples[imuIndex + 1].timestampS
                                   : std::numeric_limits<double>::infinity();
    const double nextEventTime =
        (updateIndex < contactReplayPlan.scheduledPacketIndices.size())
            ? contactReplayPlan
                  .packetDecisions[contactReplayPlan
                                       .scheduledPacketIndices[updateIndex]]
                  .timestampS
            : std::numeric_limits<double>::infinity();
    const double nextTime = std::min(nextImuTime, nextEventTime);
    if (!std::isfinite(nextTime)) {
      break;
    }
    if (nextTime - startTimestamp > maxDurationSeconds) {
      break;
    }

    const double dt = nextTime - currentTime;
    if (dt < -1e-12) {
      throw std::runtime_error(
          "Dataset timestamps are not monotonically increasing.");
    }
    if (dt > 0.0) {
      estimator->predict(heldImu.omega, heldImu.specificForce, dt);
      currentTime = nextTime;
    }

    if (nextImuTime == nextTime) {
      ++imuIndex;
      heldImu = dataset.imuSamples[imuIndex];
      logState("imu", currentTime);
    }

    while (
        updateIndex < contactReplayPlan.scheduledPacketIndices.size() &&
        std::abs(contactReplayPlan
                     .packetDecisions[contactReplayPlan
                                          .scheduledPacketIndices[updateIndex]]
                     .timestampS -
                 currentTime) < 1e-12) {
      const ContactPacketDecision& update =
          contactReplayPlan.packetDecisions
              [contactReplayPlan.scheduledPacketIndices[updateIndex]];
      estimator->processContacts(update.activeContacts);
      const ExtendedPose3d estimate = estimator->estimate();
      const Matrix footholds = footholdsFromEstimate(estimate);
      
      for (const ContactMeasurement& contact : update.activeContacts) {
        const double residual =
            contactResidualNorm(estimate, bodyPImu, contact);
        metrics.meanContactResidual += residual;
        metrics.rmsContactResidual += residual * residual;
        metrics.maxContactResidual =
            std::max(metrics.maxContactResidual, residual);
        metrics.meanHeightAbsError += std::abs(
            footholds(2, static_cast<Eigen::Index>(contact.foot)) - 0.0);
        ++metrics.contactMeasurements;
      }
      ++metrics.contactEvents;
      logState(update.status == ContactPacketStatus::kUsedTouchdown
                   ? "touchdown_contact"
                   : "periodic_contact",
               currentTime);
      ++updateIndex;
    }
    
    // ===================================================================
    // COMPUTE AND WRITE FOOT VELOCITIES AT EVERY TIMESTEP
    // (Moved outside contact update loop to get continuous data)
    // ===================================================================
    const ExtendedPose3d estimate = estimator->estimate();
    
    // Extract current estimated state for FK velocity calculation
    const NavState currentNavState = navStateFromEstimate(estimate);
    const Vector3 estimatedBodyVelocity = currentNavState.velocity();
    const Rot3 estimatedRotation = currentNavState.attitude();
    const Matrix3 R_estimated = estimatedRotation.matrix();
    
    // Extract estimated bias (if available, otherwise use initial estimate)
    const Vector3 estimatedGyroBias = imuBiasEstimate.gyroscope();
    
    size_t csvTimeIndex = findClosestTimeIndex(currentTime);
    const bool csvIndexChanged = !havePreviousCsvTimeIndex || csvTimeIndex != previousCsvTimeIndex;
    
    // 1. ESTIMATED FOOT VELOCITIES from finite-differencing FK foot positions in world frame.
    //    p_foot_world = pose.transformFrom(p_foot_body) — changes continuously unlike the
    //    graph foothold state which is nearly static between contact updates.
    const Pose3 currentPose = currentNavState.pose();
    const std::vector<Vector3> fkFootPositionsWorld =
        getFKFootPositionsWorld(csvTimeIndex, currentPose);
    std::vector<Vector3> estimatedFootVelocities(numFeet, Vector3::Zero());
    bool validEstimatedFootVelocity = false;
    if (havePreviousFKFootPositionsWorld && haveGroundTruthFootData) {
      const double dt = currentTime - previousFKFootWorldTime;
      if (dt > 1e-12) {
        for (size_t foot = 0; foot < numFeet; ++foot) {
          estimatedFootVelocities[foot] =
              (fkFootPositionsWorld[foot] - previousFKFootPositionsWorld[foot]) / dt;
        }
        validEstimatedFootVelocity = true;
      }
    }
    
    // 2. GROUND TRUTH FOOT VELOCITIES (from CSV foot position finite differencing)
    //    Only compute when the CSV index changed — GT positions are identical across
    //    multiple estimator timesteps that map to the same CSV row, so differencing
    //    them would produce zero regardless of dt.
    // Fetch current GT positions once (reused for both diff and tracking update).
    std::vector<Vector3> currentGTPositions;
    if (csvIndexChanged && haveGroundTruthFootData) {
      currentGTPositions = getGTFootPositions(csvTimeIndex);
    }

    std::vector<Vector3> gtFootVelocities(numFeet, Vector3::Zero());
    bool validGTFootVelocity = false;
    if (csvIndexChanged && havePreviousGTFootPositions && haveGroundTruthFootData) {
      const double dt = currentTime - previousGTFootTime;
      if (dt > 1e-12) {
        for (size_t foot = 0; foot < numFeet; ++foot) {
          gtFootVelocities[foot] = (currentGTPositions[foot] - previousGTFootPositions[foot]) / dt;
        }
        validGTFootVelocity = true;
      }
    }
    
    // 3. FORWARD KINEMATICS FOOT VELOCITIES (using estimated state)
    std::vector<Vector3> fkFootVelocities = getFKFootVelocities(csvTimeIndex, 
                                                                  estimatedBodyVelocity, 
                                                                  R_estimated, 
                                                                  estimatedGyroBias);
    
    // 4. CONTACT STATES (from CSV)
    std::vector<bool> contactStates = getContactStates(csvTimeIndex);
    
    // Write foot velocity row for every timestep (from the beginning)
    FootVelocityData velocityData;
    velocityData.timestampS = currentTime;
    velocityData.groundTruthVelocities = gtFootVelocities;
    velocityData.estimatedVelocities = estimatedFootVelocities;
    velocityData.fkVelocities = fkFootVelocities;
    velocityData.contactStates = contactStates;
    writeFootVelocityRow(footVelocityOutput, velocityData);
    
    // Update tracking variables for next timestep
    previousFKFootPositionsWorld = fkFootPositionsWorld;
    previousFKFootWorldTime = currentTime;
    havePreviousFKFootPositionsWorld = true;

    if (csvIndexChanged && haveGroundTruthFootData) {
      previousGTFootPositions = currentGTPositions;  // reuse already-fetched positions
      previousGTFootTime = currentTime;
      havePreviousGTFootPositions = true;
    }
    previousCsvTimeIndex = csvTimeIndex;
    havePreviousCsvTimeIndex = true;
    // ===================================================================
  }

  const auto endWallTime = std::chrono::steady_clock::now();
  metrics.wallTimeMs =
      std::chrono::duration<double, std::milli>(endWallTime - startWallTime)
          .count();
  if (metrics.contactMeasurements > 0) {
    metrics.meanContactResidual /=
        static_cast<double>(metrics.contactMeasurements);
    metrics.rmsContactResidual =
        std::sqrt(metrics.rmsContactResidual /
                  static_cast<double>(metrics.contactMeasurements));
    metrics.meanHeightAbsError /=
        static_cast<double>(metrics.contactMeasurements);
  }
  metrics.loopClosureError =
      (navStateFromEstimate(estimator->estimate()).position() -
       initialState.position())
          .norm();

  return ReplayOutputs{filterName, metrics,
                       navStateFromEstimate(estimator->estimate())};
}

void writeContactPacketUsage(const fs::path& path, const Dataset& dataset,
                             const ContactReplayPlan& plan) {
  const double startTimestamp = dataset.imuSamples.front().timestampS;
  std::vector<ContactPacketUsageRow> rows;
  for (const ContactPacketDecision& decision : plan.packetDecisions) {
    for (const ContactMeasurement& contact : decision.activeContacts) {
      rows.push_back(ContactPacketUsageRow{
          decision.eventIndex, decision.timestampS,
          decision.timestampS - startTimestamp, contact.foot,
          dataset.metadata.footNames.at(contact.foot),
          contactRowStatusName(decision.status, contact.touchdown),
          contactRowColor(decision.status, contact.touchdown)});
    }
  }
  writeContactPacketUsage(path, rows);
}

struct CommandLineArgs {
  fs::path datasetDir;
  fs::path outputDir;
  double maxDurationSeconds = std::numeric_limits<double>::infinity();
  std::optional<double> lagSeconds;
  std::optional<double> maxDeadReckoningSeconds;
  std::optional<double> sigmaAcc;
  std::optional<double> biasAccRandomWalkSigma;
  std::optional<double> biasOmegaRandomWalkSigma;
  std::optional<double> contactSigmaXY;
  std::optional<double> contactSigmaZ;
  bool useRobustContactNoise = false;
  std::optional<double> robustContactHuberK;
  bool marginalizeLeavingFoot = false;
  std::vector<std::string> filterNames;
  bool disableFullContactInitialization = true;
  bool skipPlots = false;
};

std::vector<std::string> splitCommaSeparated(const std::string& value) {
  std::vector<std::string> items;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) {
      items.push_back(item);
    }
  }
  return items;
}

std::string canonicalFilterName(const std::string& name) { return name; }

fs::path defaultDatasetDir() {
  return fs::path(__FILE__).parent_path() / "Data";
}

CommandLineArgs parseCommandLine(int argc, char* argv[]) {
  CommandLineArgs args;
  for (int index = 1; index < argc; ++index) {
    const std::string flag = argv[index];
    if ((flag == "--dataset" || flag == "-d") && index + 1 < argc) {
      args.datasetDir = argv[++index];
    } else if ((flag == "--output" || flag == "-o") && index + 1 < argc) {
      args.outputDir = argv[++index];
    } else if (flag == "--max-duration-seconds" && index + 1 < argc) {
      args.maxDurationSeconds = std::stod(argv[++index]);
    } else if (flag == "--lag-seconds" && index + 1 < argc) {
      args.lagSeconds = std::stod(argv[++index]);
    } else if (flag == "--max-dead-reckoning-seconds" && index + 1 < argc) {
      args.maxDeadReckoningSeconds = std::stod(argv[++index]);
    } else if (flag == "--sigma-acc" && index + 1 < argc) {
      args.sigmaAcc = std::stod(argv[++index]);
    } else if (flag == "--bias-acc-random-walk-sigma" && index + 1 < argc) {
      args.biasAccRandomWalkSigma = std::stod(argv[++index]);
    } else if (flag == "--bias-omega-random-walk-sigma" && index + 1 < argc) {
      args.biasOmegaRandomWalkSigma = std::stod(argv[++index]);
    } else if (flag == "--contact-sigma-xy" && index + 1 < argc) {
      args.contactSigmaXY = std::stod(argv[++index]);
    } else if (flag == "--contact-sigma-z" && index + 1 < argc) {
      args.contactSigmaZ = std::stod(argv[++index]);
    } else if (flag == "--robust-contact-noise") {
      args.useRobustContactNoise = true;
    } else if (flag == "--robust-contact-huber-k" && index + 1 < argc) {
      args.robustContactHuberK = std::stod(argv[++index]);
    } else if (flag == "--marginalize-leaving-foot") {
      args.marginalizeLeavingFoot = true;
    } else if (flag == "--no-marginalize-leaving-foot" ||
               flag == "--disable-marginalize-leaving-foot") {
      args.marginalizeLeavingFoot = false;
    } else if (flag == "--skip-plots") {
      args.skipPlots = true;
    } else if (flag == "--variants" && index + 1 < argc) {
      args.filterNames = splitCommaSeparated(argv[++index]);
    } else if (flag == "--disable-full-contact-initialization") {
      args.disableFullContactInitialization = true;
    } else if (flag == "--help" || flag == "-h") {
      std::cout << "Usage: LeggedEstimatorReplayExample [--dataset <dir>] "
                   "[--output <dir>] "
                   "[--max-duration-seconds <seconds>] "
                   "[--lag-seconds <seconds>] "
                   "[--max-dead-reckoning-seconds <seconds>] "
                   "[--sigma-acc <m/s^2>] "
                   "[--bias-acc-random-walk-sigma <m/s^2/sqrt(s)>] "
                   "[--bias-omega-random-walk-sigma <rad/s/sqrt(s)>] "
                   "[--contact-sigma-xy <meters>] "
                   "[--contact-sigma-z <meters>] "
                   "[--robust-contact-noise] "
                   "[--robust-contact-huber-k <threshold>] "
                   "[--marginalize-leaving-foot|--no-marginalize-leaving-foot] "
                   "[--variants <comma,separated,list>] "
                   "[--disable-full-contact-initialization]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown command line argument: " + flag);
    }
  }

  if (args.datasetDir.empty()) {
    args.datasetDir = defaultDatasetDir();
  }
  if (args.outputDir.empty()) {
    args.outputDir = fs::path(__FILE__).parent_path() / "results";
  }
  if (args.filterNames.empty()) {
    args.filterNames = {"fixed_lag_single_bias"};
  }
  for (std::string& filterName : args.filterNames) {
    filterName = canonicalFilterName(filterName);
  }
  return args;
}

ReplayConfig makeReplayConfig(const CommandLineArgs& args) {
  ReplayConfig replayConfig;
  if (args.lagSeconds) {
    replayConfig.lagSeconds = *args.lagSeconds;
  }
  if (args.maxDeadReckoningSeconds) {
    replayConfig.maxDeadReckoningSeconds = *args.maxDeadReckoningSeconds;
  }
  if (args.sigmaAcc) {
    replayConfig.sigmaAcc = *args.sigmaAcc;
  }
  if (args.biasAccRandomWalkSigma) {
    replayConfig.biasAccRandomWalkSigma = *args.biasAccRandomWalkSigma;
  }
  if (args.biasOmegaRandomWalkSigma) {
    replayConfig.biasOmegaRandomWalkSigma = *args.biasOmegaRandomWalkSigma;
  }
  if (args.contactSigmaXY || args.contactSigmaZ) {
    const double sigmaXY = args.contactSigmaXY.value_or(
        std::sqrt(replayConfig.contactCovariance(0, 0)));
    const double sigmaZ = args.contactSigmaZ.value_or(
        std::sqrt(replayConfig.contactCovariance(2, 2)));
    replayConfig.contactCovariance =
        (Vector3(sigmaXY * sigmaXY, sigmaXY * sigmaXY, sigmaZ * sigmaZ))
            .asDiagonal();
  }
  replayConfig.useRobustContactNoise = args.useRobustContactNoise;
  if (args.robustContactHuberK) {
    replayConfig.robustContactHuberK = *args.robustContactHuberK;
  }
  replayConfig.marginalizeLeavingFoot = args.marginalizeLeavingFoot;
  return replayConfig;
}

}  // namespace
}  // namespace gtsam

int main(int argc, char* argv[]) {
  using namespace gtsam;
  const CommandLineArgs args = parseCommandLine(argc, argv);
  const ReplayConfig replayConfig = makeReplayConfig(args);
  const Dataset dataset = loadDataset(args.datasetDir);
  
  // Load ground truth data from CSV (used for both initialization and foot velocity calculations)
  const GroundTruthData gtData = loadGroundTruthData(args.datasetDir);
  
  // const InitialBiasEstimate imuBiasEstimate = estimateInitialImuBias(dataset);
  const InitialBiasEstimate imuBiasEstimate = {imuBias::ConstantBias(), "zero initialization", 0};
  const double maxDeadReckoningSeconds = replayConfig.maxDeadReckoningSeconds;
  if (maxDeadReckoningSeconds > 0.0 && !dataset.metadata.denseContactStream) {
    throw std::runtime_error(
        "Synthetic contact updates require a dense contact-state dataset.");
  }
  const ContactReplayPlan contactReplayPlan =
      buildContactReplayPlan(dataset, maxDeadReckoningSeconds);
  fs::create_directories(args.outputDir);
  std::cout << "Estimated startup IMU bias (" << imuBiasEstimate.source
            << ", samples=" << imuBiasEstimate.sampleCount
            << "): acc=" << imuBiasEstimate.bias.accelerometer().transpose()
            << " gyro=" << imuBiasEstimate.bias.gyroscope().transpose() << '\n';
  std::cout << "Dense contact stream: "
            << (dataset.metadata.denseContactStream ? "yes" : "no")
            << ", max_dead_reckoning_seconds="
            << toString(maxDeadReckoningSeconds, 3) << '\n';
  std::cout << "Lag seconds: " << toString(replayConfig.lagSeconds, 3) << '\n';
  std::cout << "Contact sigmas [xy z]: "
            << toString(std::sqrt(replayConfig.contactCovariance(0, 0)), 3)
            << ' '
            << toString(std::sqrt(replayConfig.contactCovariance(2, 2)), 3)
            << '\n';
  std::cout << "Robust contact noise: "
            << (replayConfig.useRobustContactNoise ? "yes" : "no") << '\n';
  std::cout << "Robust contact Huber k: "
            << toString(replayConfig.robustContactHuberK, 3) << '\n';
  std::cout << "Marginalize leaving foot: "
            << (replayConfig.marginalizeLeavingFoot ? "yes" : "no") << '\n';
  std::cout << "Contact state value: " << dataset.metadata.contactStateValue
            << " (treated as contact), timestamp_source="
            << dataset.metadata.timestampSource << '\n';
  std::cout << "Shared contact schedule: "
            << contactReplayPlan.scheduledPacketIndices.size()
            << " used updates from " << contactReplayPlan.packetDecisions.size()
            << " available packets\n";
  writeContactPacketUsage(args.outputDir / "contact_packet_usage.csv", dataset,
                          contactReplayPlan);

  for (const std::string& filterName : args.filterNames) {
    std::ofstream trajectoryOutput(args.outputDir /
                                   (filterName + "_trajectory.csv"));
    if (!trajectoryOutput) {
      throw std::runtime_error("Unable to open trajectory output for writing.");
    }
    writeTrajectoryHeader(trajectoryOutput);

    std::ofstream metricsOutput(args.outputDir / (filterName + "_metrics.csv"));
    if (!metricsOutput) {
      throw std::runtime_error("Unable to open metrics output for writing.");
    }
    writeMetricsHeader(metricsOutput);

    std::ofstream footVelocityOutput(args.outputDir / 
                                     (filterName + "_foot_velocities.csv"));
    if (!footVelocityOutput) {
      throw std::runtime_error("Unable to open foot velocity output for writing.");
    }
    writeFootVelocityHeader(footVelocityOutput, dataset.metadata.footNames);

    const ReplayOutputs outputs = replayFilter(
        filterName, dataset, contactReplayPlan, replayConfig, trajectoryOutput,
        footVelocityOutput, args.maxDurationSeconds, 
        args.disableFullContactInitialization,
        imuBiasEstimate.bias, gtData.initialState, gtData.fullData, gtData.dataLoaded);
    writeMetricsRow(metricsOutput, outputs);
    std::cout << filterName
              << ": wall_time_ms=" << toString(outputs.metrics.wallTimeMs, 3)
              << ", rms_contact_residual="
              << toString(outputs.metrics.rmsContactResidual, 6)
              << ", path_length=" << toString(outputs.metrics.pathLength, 3)
              << ", loop_closure_error="
              << toString(outputs.metrics.loopClosureError, 3) << '\n';
    std::cout << "  saved "
              << (args.outputDir / (filterName + "_trajectory.csv")).string()
              << '\n';
    std::cout << "  saved "
              << (args.outputDir / (filterName + "_metrics.csv")).string()
              << '\n';
    std::cout << "  saved "
              << (args.outputDir / (filterName + "_foot_velocities.csv")).string()
              << '\n';
  }
  
  // Automatically generate foot velocity plots after processing all filters (unless disabled)
  if (!args.skipPlots) {
    std::cout << "\n=== Generating foot velocity plots ===\n";
    for (const std::string& filterName : args.filterNames) {
      std::string csvPath = (args.outputDir / (filterName + "_foot_velocities.csv")).string();
      // Get the directory containing this source file for the Python script location
      std::string scriptDir = fs::path(__FILE__).parent_path().string();
      std::string plotCommand = "cd \"" + scriptDir + "\" && python3 plotFootVelocities.py \"" + csvPath + "\"";
      std::cout << "Running: " << plotCommand << std::endl;
      int plotResult = std::system(plotCommand.c_str());
      if (plotResult != 0) {
        std::cerr << "Warning: Plot generation failed for " << filterName << " (exit code: " << plotResult << ")\n";
      }
    }
    std::cout << "=== Plot generation complete ===\n";
  } else {
    std::cout << "\n(Skipping plot generation - use without --skip-plots to enable)\n";
  }
  
  return 0;
}