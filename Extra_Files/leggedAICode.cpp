/* ============================================================================
 * mainSmoother.cpp
 * G1 Biped State Estimation — LeggedCombinedFixedLagSmoother
 *
 * Data flow (mirrors leggedExampleUsage.cpp):
 *   CSV  →  RobotDataset  →  ContactReplayPlan  →  interleaved predict / processContacts
 *
 * Run with real data:
 *   ./mainSmoother --csv ../Data/G1LoggingData.csv --output ./smoother_outputs
 *
 * Run with 5-sample synthetic test:
 *   ./mainSmoother --test
 *
 * TODOs (marked below):
 *   [TODO-CSV]       Full CSV parser body (loadDatasetFromCSV)
 *   [TODO-PINOCCHIO] Replace world-frame foot FK with Pinocchio body-frame FK
 *   [TODO-BODY_P_IMU] Set body_P_imu from URDF (G1 imuFrame_id = 3)
 *   [TODO-NOISE]     Tune SmootherConfig noise params for G1
 * ========================================================================= */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/NavState.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace gtsam;

// ============================================================================
// Data Structures
// ============================================================================

struct ImuSample {
  double timestampS = 0.0;
  Vector3 omega = Vector3::Zero();          // gyroscope  (rad/s)  in body frame
  Vector3 specificForce = Vector3::Zero();  // accelerometer (m/s²) in body frame
};

// Is this gravity compensated?  No, it's the raw specific force measurement from the IMU, which
// includes gravity.  The smoother will handle gravity compensation internally based on the current attitude estimate and the configured gravity vector.


struct ContactEvent {
  size_t index = 0;
  double timestampS = 0.0;
  std::vector<ContactMeasurement> activeContacts;
};

// ContactMeasurement is defined in LeggedEstimator.h as:
// struct ContactMeasurement {
//   size_t foot;           // foot index (e.g., 0 for left foot, 1 for right foot)
//   Vector3 bodyPoint;     // contact point position in body frame
//   bool touchdown;        // true if this contact measurement is a touchdown event
// };

struct RobotDataset {
  std::vector<ImuSample> imuSamples;
  std::vector<ContactEvent> contactEvents;
  NavState initialNavState;             // from first CSV row
  bool denseContactStream = true;      // true if contact logged at every IMU step
};

// ============================================================================
// SmootherConfig
// ============================================================================

struct SmootherConfig {
  Vector3 gravity = Vector3(0.0, 0.0, -9.81);

  // IMU noise  (G1_config: gyro=1e-4, accel=1e-2 → sigmas = sqrt)
  double sigmaGyro        = 1e-2;   // [TODO-NOISE] tune for G1
  double sigmaAcc         = 1e-1;   // [TODO-NOISE]
  double sigmaIntegration = 1e-3;

  // Bias random walk
  double biasAccRandomWalkSigma   = 5e-3;
  double biasOmegaRandomWalkSigma = 1e-4;

  // Contact noise [x, y, z] variances
  Matrix3 contactCovariance =
      (Vector3(0.03 * 0.03, 0.03 * 0.03, 0.02 * 0.02)).asDiagonal();

  // Foothold uncertainty
  double footholdInitSigma    = 0.5;
  double footholdProcessSigma = 1e-4;
  double heightPriorSigma     = 0.15;

  // Smoother window
  double lagSeconds               = 1.0;
  double maxDeadReckoningSeconds  = 0.1;

  // [TODO-BODY_P_IMU] Set from URDF.  G1 imuFrame_id = 3 in Pinocchio model.
  Pose3 body_P_imu = Pose3(Rot3(), Point3(0.0, 0.0, 0.0));

  // Initial base covariance diagonal [rot(3), pos(3), vel(3)]
  Vector initialBaseCovDiag =
      (Vector(9) << 1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05)
          .finished();

  bool useRobustContactNoise      = false;
  double robustContactHuberK      = 2.0;
  bool marginalizeLeavingFoot     = true;
  bool useFullContactInitialization = true;
};

// ============================================================================
// Contact Replay Plan  (identical logic to leggedExampleUsage.cpp)
// ============================================================================

enum class ContactPacketStatus { kIgnored, kUsedTouchdown, kUsedPeriodic };

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

static bool hasTouchdown(const std::vector<ContactMeasurement>& contacts) {
  return std::any_of(contacts.begin(), contacts.end(),
                     [](const ContactMeasurement& c) { return c.touchdown; });
}

static ContactReplayPlan buildContactReplayPlan(
    const RobotDataset& dataset, double maxDeadReckoningSeconds) {
  ContactReplayPlan plan;
  plan.packetDecisions.reserve(dataset.contactEvents.size());

  double lastUpdateTime = -std::numeric_limits<double>::infinity();

  for (const ContactEvent& event : dataset.contactEvents) {
    ContactPacketDecision decision;
    decision.eventIndex    = event.index;
    decision.timestampS    = event.timestampS;
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

// ============================================================================
// Initial Bias Estimation  (mirrors leggedExampleUsage.cpp)
// ============================================================================

struct InitialBiasEstimate {
  imuBias::ConstantBias bias;
  std::string source;
  size_t sampleCount = 0;
};

static bool samplesLookStationary(const std::vector<ImuSample>& samples,
                                  double gravityMagnitude) {
  if (samples.empty()) return false;
  double meanOmegaNorm = 0.0, meanAccelNorm = 0.0;
  for (const ImuSample& s : samples) {
    meanOmegaNorm += s.omega.norm();
    meanAccelNorm += s.specificForce.norm();
  }
  meanOmegaNorm /= static_cast<double>(samples.size());
  meanAccelNorm /= static_cast<double>(samples.size());
  return meanOmegaNorm < 1e-2 &&
         std::abs(meanAccelNorm - gravityMagnitude) < 0.15;
}

static imuBias::ConstantBias estimateBiasFromSamples(
    const std::vector<ImuSample>& samples, const NavState& initialNavState,
    const SmootherConfig& cfg) {
  Vector3 meanOmega = Vector3::Zero(), meanForce = Vector3::Zero();
  for (const ImuSample& s : samples) {
    meanOmega += s.omega;
    meanForce += s.specificForce;
  }
  meanOmega /= static_cast<double>(samples.size());
  meanForce /= static_cast<double>(samples.size());
  const Vector3 expectedForce = initialNavState.attitude().unrotate(-cfg.gravity);
  return imuBias::ConstantBias(meanForce - expectedForce, meanOmega);
}

static InitialBiasEstimate estimateInitialBias(const RobotDataset& dataset,
                                               const SmootherConfig& cfg) {
  const double gravMag = cfg.gravity.norm();
  constexpr double kMaxWindowSec  = 4.0;
  constexpr double kMaxOmegaNorm  = 1e-2;
  constexpr double kMaxAccelErr   = 0.2;
  const double startT = dataset.imuSamples.front().timestampS;

  std::vector<ImuSample> samples;
  for (const ImuSample& s : dataset.imuSamples) {
    if (s.timestampS - startT > kMaxWindowSec) break;
    const bool stationary =
        s.omega.norm() <= kMaxOmegaNorm &&
        std::abs(s.specificForce.norm() - gravMag) <= kMaxAccelErr;
    if (!stationary && !samples.empty()) break;
    if (stationary) samples.push_back(s);
  }
  if (samples.empty()) {
    const size_t n = std::min<size_t>(dataset.imuSamples.size(), 200);
    samples.assign(dataset.imuSamples.begin(), dataset.imuSamples.begin() + n);
  }

  const std::string source =
      samplesLookStationary(samples, gravMag) ? "pre-init static window"
                                              : "fallback (first samples)";
  return {estimateBiasFromSamples(samples, dataset.initialNavState, cfg),
          source, samples.size()};
}

// ============================================================================
// Helpers: NavState / Footholds from estimate
// (EstimateNavState is protected in LeggedEstimator, so replicate the logic)
// ============================================================================

static NavState navStateFromEstimate(const ExtendedPose3d& est) {
  return NavState(est.rotation(), est.x(0), est.x(1));
}

// ============================================================================
// Parameter Builder
// ============================================================================

static LeggedEstimatorParams makeEstimatorParams(
    const SmootherConfig& cfg, const imuBias::ConstantBias& bias) {
  // LeggedCombinedFixedLagSmoother wraps PreintegrationParams internally into
  // PreintegrationCombinedParams using biasAcc/OmegaRandomWalkSigma.
  auto preint = std::make_shared<PreintegrationParams>(cfg.gravity);
  preint->gyroscopeCovariance =
      Matrix3::Identity() * (cfg.sigmaGyro * cfg.sigmaGyro);
  preint->accelerometerCovariance =
      Matrix3::Identity() * (cfg.sigmaAcc * cfg.sigmaAcc);
  preint->integrationCovariance =
      Matrix3::Identity() * (cfg.sigmaIntegration * cfg.sigmaIntegration);

  LeggedEstimatorParams params;
  params.preintegrationParams         = preint;
  params.imuBias                      = bias;
  params.body_P_imu                   = cfg.body_P_imu;
  params.contactCovariance            = cfg.contactCovariance;
  params.footholdInitSigma            = cfg.footholdInitSigma;
  params.footholdProcessSigma         = cfg.footholdProcessSigma;
  params.heightPriorSigma             = cfg.heightPriorSigma;
  params.useRobustContactNoise        = cfg.useRobustContactNoise;
  params.robustContactHuberK          = cfg.robustContactHuberK;
  params.marginalizeLeavingFoot       = cfg.marginalizeLeavingFoot;
  params.useFullContactInitialization = cfg.useFullContactInitialization;
  params.biasAccRandomWalkSigma       = cfg.biasAccRandomWalkSigma;
  params.biasOmegaRandomWalkSigma     = cfg.biasOmegaRandomWalkSigma;
  return params;
}

static Matrix9 makeBaseCovariance(const SmootherConfig& cfg) {
  Matrix9 cov = Matrix9::Zero();
  cov.diagonal() = cfg.initialBaseCovDiag;
  return cov;
}

// ============================================================================
// Trajectory Output
// ============================================================================

static void writeTrajectoryHeader(std::ofstream& out) {
  out << "kind,timestamp_s,"
         "pos_x,pos_y,pos_z,"
         "vel_x,vel_y,vel_z,"
         "quat_w,quat_x,quat_y,quat_z,"
         "bias_acc_x,bias_acc_y,bias_acc_z,"
         "bias_gyro_x,bias_gyro_y,bias_gyro_z\n";
}

static void writeTrajectoryRow(std::ofstream& out, const std::string& kind,
                               double ts, const NavState& state,
                               const imuBias::ConstantBias& bias) {
  const auto q  = state.attitude().toQuaternion();
  const Vector3& p  = state.position();
  const Vector3& v  = state.velocity();
  const Vector3& ba = bias.accelerometer();
  const Vector3& bg = bias.gyroscope();
  out << std::fixed << std::setprecision(9)
      << kind  << "," << ts
      << "," << p.x()  << "," << p.y()  << "," << p.z()
      << "," << v.x()  << "," << v.y()  << "," << v.z()
      << "," << q.w()  << "," << q.x()  << "," << q.y()  << "," << q.z()
      << "," << ba.x() << "," << ba.y() << "," << ba.z()
      << "," << bg.x() << "," << bg.y() << "," << bg.z()
      << "\n";
}

// ============================================================================
// Full Contact Initialization (from reference leggedExampleUsage.cpp)
// ============================================================================
//
// Computes initial NavState from contact measurements by:
//   1. Computing ground plane normal from contact points
//   2. Extracting roll/pitch from normal
//   3. Computing height from contacts
//   4. Setting yaw=0 (no information from contacts)

static NavState fullContactInitializationNavState(
    const SmootherConfig& cfg,
    const std::vector<ContactMeasurement>& activeContacts) {
  // Transform all contact points to IMU frame
  Matrix imuContacts(3, static_cast<Eigen::Index>(activeContacts.size()));
  for (size_t index = 0; index < activeContacts.size(); ++index) {
    imuContacts.col(static_cast<Eigen::Index>(index)) =
        cfg.body_P_imu.transformTo(
            Point3(activeContacts[index].bodyPoint));
  }

  // Compute ground plane normal using SVD
  const Vector3 centroid = imuContacts.rowwise().mean();
  const Matrix centered = imuContacts.colwise() - centroid;
  Vector3 normal =
      Eigen::JacobiSVD<Matrix>(centered, Eigen::ComputeFullU)
          .matrixU()
          .col(2);
  if (normal.z() < 0.0) {
    normal = -normal;  // Ensure normal points up
  }

  // Extract roll and pitch from normal (yaw = 0)
  const double roll = std::atan2(normal.y(), normal.z());
  const double pitch = std::asin(-normal.x());
  const Rot3 attitude = Rot3::Ypr(0.0, pitch, roll);

  // Compute height from contact points
  double height = 0.0;
  for (const ContactMeasurement& contact : activeContacts) {
    const Point3 measurement =
        cfg.body_P_imu.transformTo(Point3(contact.bodyPoint));
    height -= attitude.matrix().row(2).dot(measurement);
  }
  height /= static_cast<double>(activeContacts.size());

  return NavState(attitude, Point3(0.0, 0.0, height), Vector3::Zero());
}

// ============================================================================
// CSV Loader
// ============================================================================
//
// [TODO-CSV] This function reads the G1LoggingData.csv columns:
//
//   timestamp          → ImuSample::timestampS / ContactEvent::timestampS
//   gyro_body_{x,y,z}  → ImuSample::omega        (rad/s, body frame)
//   acc_body_{x,y,z}   → ImuSample::specificForce (m/s², body frame)
//   lfoot-contact      → ContactMeasurement::touchdown (detect 0→1 edge)
//   rfoot-contact      → ContactMeasurement::touchdown
//   lfoot_pos_{x,y,z}  → ContactMeasurement::bodyPoint for foot 0
//   rfoot_pos_{x,y,z}  → ContactMeasurement::bodyPoint for foot 1
//   pos_{x,y,z}        → initial NavState position
//   quat_{w,i,j,k}     → initial NavState rotation  (Eigen/GTSAM: w,x,y,z)
//   vel_{x,y,z}        → initial NavState velocity
//
// [TODO-PINOCCHIO] lfoot_pos / rfoot_pos in the CSV are world-frame positions
//   from FK + state estimate.  They are transformed here to body frame via:
//       p_body = R_wb^T * (p_world - p_robot)
//   Replace this transform with a direct Pinocchio FK call that returns the
//   contact frame position already expressed in the body (pelvis) frame.

static std::vector<std::string> splitCSV(const std::string& line) {
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) tokens.push_back(tok);
  return tokens;
}

RobotDataset loadDatasetFromCSV(const std::string& csvPath) {
  std::ifstream file(csvPath);
  if (!file.is_open())
    throw std::runtime_error("Cannot open CSV: " + csvPath);

  RobotDataset dataset;
  dataset.denseContactStream = true;  // G1 logs contact at every IMU timestep

  // --- Parse header to find column indices ---
  std::string headerLine;
  std::getline(file, headerLine);
  const auto headers = splitCSV(headerLine);

  auto colIdx = [&](const std::string& name) -> int {
    for (int i = 0; i < static_cast<int>(headers.size()); ++i)
      if (headers[i] == name) return i;
    throw std::runtime_error("CSV column not found: '" + name + "'");
  };

  const int iTs  = colIdx("timestamp");
  const int iGx  = colIdx("gyro_body_x"),  iGy  = colIdx("gyro_body_y"),
            iGz  = colIdx("gyro_body_z");
  const int iAx  = colIdx("acc_body_x"),   iAy  = colIdx("acc_body_y"),
            iAz  = colIdx("acc_body_z");
  const int iLC  = colIdx("lfoot-contact"),  iRC  = colIdx("rfoot-contact");
  const int iLFx = colIdx("lfoot_pos_x"), iLFy = colIdx("lfoot_pos_y"),
            iLFz = colIdx("lfoot_pos_z");
  const int iRFx = colIdx("rfoot_pos_x"), iRFy = colIdx("rfoot_pos_y"),
            iRFz = colIdx("rfoot_pos_z");
  const int iPx  = colIdx("pos_x"),  iPy  = colIdx("pos_y"),  iPz  = colIdx("pos_z");
  const int iQw  = colIdx("quat_w"), iQi  = colIdx("quat_i"),
            iQj  = colIdx("quat_j"), iQk  = colIdx("quat_k");
  const int iVx  = colIdx("vel_x"),  iVy  = colIdx("vel_y"),  iVz  = colIdx("vel_z");

  bool firstRow    = true;
  bool prevLC = false, prevRC = false;
  size_t eventIdx  = 0;
  std::string line;
  bool initialNavStateSet = false;

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    const auto tok = splitCSV(line);
    const auto getd = [&](int i) {
      return std::stod(tok[static_cast<size_t>(i)]);
    };

    const double ts = getd(iTs);

    // IMU sample
    ImuSample imu;
    imu.timestampS    = ts;
    imu.omega         = Vector3(getd(iGx), getd(iGy), getd(iGz));
    imu.specificForce = Vector3(getd(iAx), getd(iAy), getd(iAz));
    dataset.imuSamples.push_back(imu);

    // Parse rotation and position from CSV for body-to-world transform
    const Rot3 R_wb = Rot3::Quaternion(getd(iQw), getd(iQi), getd(iQj), getd(iQk));
    const Vector3 p_robot(getd(iPx), getd(iPy), getd(iPz));
    const Vector3 vel(getd(iVx), getd(iVy), getd(iVz));

    // [TODO-PINOCCHIO] Replace world→body transform with Pinocchio FK
    const Vector3 lFoot_body =
        R_wb.unrotate(Vector3(getd(iLFx), getd(iLFy), getd(iLFz)) - p_robot);
    const Vector3 rFoot_body =
        R_wb.unrotate(Vector3(getd(iRFx), getd(iRFy), getd(iRFz)) - p_robot);

    const bool lC = (getd(iLC) > 0.5);
    const bool rC = (getd(iRC) > 0.5);

    // Build contact event for this timestep
    std::vector<ContactMeasurement> activeContacts;
    if (lC) {
      ContactMeasurement cm;
      cm.foot      = 0;
      cm.bodyPoint = lFoot_body;
      cm.touchdown = (!prevLC);
      activeContacts.push_back(cm);
    }
    if (rC) {
      ContactMeasurement cm;
      cm.foot      = 1;
      cm.bodyPoint = rFoot_body;
      cm.touchdown = (!prevRC);
      activeContacts.push_back(cm);
    }
    if (!activeContacts.empty()) {
      ContactEvent ev;
      ev.index          = eventIdx++;
      ev.timestampS     = ts;
      ev.activeContacts = std::move(activeContacts);
      dataset.contactEvents.push_back(std::move(ev));
    }

    // Initialize NavState from first full contact event
    if (firstRow && !initialNavStateSet) {
      if (activeContacts.size() == 2) {  // Both feet in contact
        SmootherConfig cfg;  // Use default config for body_P_imu
        dataset.initialNavState = 
            fullContactInitializationNavState(cfg, activeContacts);
        initialNavStateSet = true;
      }
      firstRow = false;
    }

    prevLC = lC;
    prevRC = rC;
  }

  // Fallback to CSV values if no full contact event found
  if (!initialNavStateSet) {
    std::ifstream fileFallback(csvPath);
    std::getline(fileFallback, headerLine);  // skip header
    std::getline(fileFallback, line);
    if (!line.empty()) {
      const auto tok = splitCSV(line);
      const auto getd = [&](int i) {
        return std::stod(tok[static_cast<size_t>(i)]);
      };
      const Rot3 R_wb = Rot3::Quaternion(getd(iQw), getd(iQi), getd(iQj), getd(iQk));
      const Vector3 p_robot(getd(iPx), getd(iPy), getd(iPz));
      const Vector3 vel(getd(iVx), getd(iVy), getd(iVz));
      dataset.initialNavState = NavState(R_wb, Point3(p_robot), vel);
    }
  }

  return dataset;
}

// ============================================================================
// Test Data Generator — 5 synthetic samples (no CSV needed)
// ============================================================================

static RobotDataset generateTestDataset() {
  RobotDataset dataset;
  dataset.denseContactStream = true;

  constexpr double dt = 0.02;  // 50 Hz
  constexpr int    N  = 5;

  // Standing still at z = 0.80 m
  const NavState state0(Rot3(), Point3(0.0, 0.0, 0.80), Vector3::Zero());
  dataset.initialNavState = state0;

  // Stationary IMU: omega ≈ 0, specific force ≈ +g in body frame (z-up)
  const Vector3 nomOmega = Vector3::Zero();
  const Vector3 nomForce = Vector3(0.0, 0.0, 9.81);

  // Fixed foot positions in body frame (G1 approximate stance geometry)
  const Vector3 lFootBody( 0.0,  0.10, -0.80);
  const Vector3 rFootBody( 0.0, -0.10, -0.80);

  bool prevLC = false, prevRC = false;  // first step will be a touchdown

  for (int i = 0; i < N; ++i) {
    const double ts = i * dt;

    // IMU — tiny perturbations to avoid degenerate covariances
    ImuSample imu;
    imu.timestampS    = ts;
    imu.omega         = nomOmega + Vector3(1e-4 * (i % 3 - 1), 1e-4 * (i % 2), 0.0);
    imu.specificForce = nomForce + Vector3(2e-3 * (i % 2), -1e-3 * (i % 3), 0.0);
    dataset.imuSamples.push_back(imu);

    // Both feet in contact; first sample fires touchdowns
    const bool lC = true, rC = true;

    std::vector<ContactMeasurement> activeContacts;

    ContactMeasurement lCM;
    lCM.foot      = 0;
    lCM.bodyPoint = lFootBody;
    lCM.touchdown = (!prevLC);
    activeContacts.push_back(lCM);

    ContactMeasurement rCM;
    rCM.foot      = 1;
    rCM.bodyPoint = rFootBody;
    rCM.touchdown = (!prevRC);
    activeContacts.push_back(rCM);

    ContactEvent ev;
    ev.index          = static_cast<size_t>(i);
    ev.timestampS     = ts;
    ev.activeContacts = std::move(activeContacts);
    dataset.contactEvents.push_back(std::move(ev));

    prevLC = lC;
    prevRC = rC;
  }

  return dataset;
}

// ============================================================================
// Main Smoother Loop
// ============================================================================

struct SmootherOutputs {
  double wallTimeMs   = 0.0;
  size_t contactEvents = 0;
  size_t imuSteps      = 0;
  double pathLength    = 0.0;
  NavState finalNavState;
  imuBias::ConstantBias finalBias;
};

static SmootherOutputs runSmoother(const RobotDataset& dataset,
                                   const ContactReplayPlan& replayPlan,
                                   const SmootherConfig& cfg,
                                   const imuBias::ConstantBias& initialBias,
                                   std::ofstream& trajectoryOut,
                                   double maxDurationSeconds) {
  const LeggedEstimatorParams params = makeEstimatorParams(cfg, initialBias);
  const Matrix9 baseCov              = makeBaseCovariance(cfg);
  const Matrix  footholds            = Matrix::Zero(3, 2);  // 2 feet
  const std::vector<std::string> footNames = {"left_foot", "right_foot"};

  LeggedCombinedFixedLagSmoother smoother(
      dataset.initialNavState, footholds, baseCov, params,
      cfg.lagSeconds, footNames);

  SmootherOutputs out;
  NavState prevLoggedState = dataset.initialNavState;
  bool havePrev = false;

  const double startTimestamp = dataset.imuSamples.front().timestampS;

  // Helper: extract current state, log it, accumulate path length
  auto logState = [&](const std::string& kind, double ts) {
    const ExtendedPose3d est         = smoother.estimate();
    const imuBias::ConstantBias bias = smoother.estimateBias();
    const NavState state             = navStateFromEstimate(est);
    writeTrajectoryRow(trajectoryOut, kind, ts, state, bias);
    if (havePrev)
      out.pathLength += (state.position() - prevLoggedState.position()).norm();
    prevLoggedState = state;
    havePrev = true;
  };

  const auto wallStart = std::chrono::steady_clock::now();
  writeTrajectoryRow(trajectoryOut, "start", startTimestamp,
                     dataset.initialNavState, initialBias);

  size_t imuIdx    = 0;
  size_t updateIdx = 0;
  ImuSample heldImu = dataset.imuSamples.front();
  double currentTime = heldImu.timestampS;

  while (true) {
    // Next IMU timestamp
    const double nextImuTime =
        (imuIdx + 1 < dataset.imuSamples.size())
            ? dataset.imuSamples[imuIdx + 1].timestampS
            : std::numeric_limits<double>::infinity();

    // Next scheduled contact update timestamp
    const double nextEventTime =
        (updateIdx < replayPlan.scheduledPacketIndices.size())
            ? replayPlan
                  .packetDecisions[replayPlan.scheduledPacketIndices[updateIdx]]
                  .timestampS
            : std::numeric_limits<double>::infinity();

    const double nextTime = std::min(nextImuTime, nextEventTime);
    if (!std::isfinite(nextTime)) break;
    if (nextTime - startTimestamp > maxDurationSeconds) break;

    const double dt = nextTime - currentTime;
    if (dt < -1e-12)
      throw std::runtime_error("Non-monotonic timestamps in dataset.");

    // Predict forward with held IMU sample
    if (dt > 0.0) {
      smoother.predict(heldImu.omega, heldImu.specificForce, dt);
      currentTime = nextTime;
    }

    // Advance to the next IMU sample
    if (nextImuTime == nextTime) {
      ++imuIdx;
      heldImu = dataset.imuSamples[imuIdx];
      ++out.imuSteps;
      logState("imu", currentTime);
    }

    // Apply all scheduled contact updates at this time
    while (updateIdx < replayPlan.scheduledPacketIndices.size() &&
           std::abs(replayPlan
                        .packetDecisions[replayPlan.scheduledPacketIndices[updateIdx]]
                        .timestampS -
                    currentTime) < 1e-12) {
      const ContactPacketDecision& upd =
          replayPlan
              .packetDecisions[replayPlan.scheduledPacketIndices[updateIdx]];
      smoother.processContacts(upd.activeContacts);
      ++out.contactEvents;
      logState(upd.status == ContactPacketStatus::kUsedTouchdown
                   ? "touchdown_contact"
                   : "periodic_contact",
               currentTime);
      ++updateIdx;
    }
  }

  const auto wallEnd = std::chrono::steady_clock::now();
  out.wallTimeMs =
      std::chrono::duration<double, std::milli>(wallEnd - wallStart).count();

  const ExtendedPose3d finalEst = smoother.estimate();
  out.finalNavState = navStateFromEstimate(finalEst);
  out.finalBias     = smoother.estimateBias();

  return out;
}

// ============================================================================
// Command-Line Arguments
// ============================================================================

struct Args {
  std::string csvPath;
  fs::path    outputDir             = fs::current_path() / "smoother_outputs";
  double      maxDurationSeconds    = std::numeric_limits<double>::infinity();
  double      lagSeconds            = 1.0;
  double      maxDeadReckoning      = 0.1;
  bool        useTestData           = false;
};

static Args parseArgs(int argc, char* argv[]) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if ((flag == "--csv" || flag == "-c") && i + 1 < argc)
      args.csvPath = argv[++i];
    else if ((flag == "--output" || flag == "-o") && i + 1 < argc)
      args.outputDir = argv[++i];
    else if (flag == "--max-duration-seconds" && i + 1 < argc)
      args.maxDurationSeconds = std::stod(argv[++i]);
    else if (flag == "--lag-seconds" && i + 1 < argc)
      args.lagSeconds = std::stod(argv[++i]);
    else if (flag == "--max-dead-reckoning-seconds" && i + 1 < argc)
      args.maxDeadReckoning = std::stod(argv[++i]);
    else if (flag == "--test")
      args.useTestData = true;
    else if (flag == "--help" || flag == "-h") {
      std::cout
          << "Usage: mainSmoother [options]\n"
             "\n"
             "  --csv  <path>    Path to G1LoggingData.csv\n"
             "  --output <dir>   Output directory (default: ./smoother_outputs)\n"
             "  --max-duration-seconds <s>\n"
             "  --lag-seconds <s>               Smoother window (default: 1.0)\n"
             "  --max-dead-reckoning-seconds <s> Max gap between contact updates\n"
             "  --test           Run on 5-sample synthetic data (no CSV needed)\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + flag);
    }
  }
  // Default to test data if no CSV provided
  if (args.csvPath.empty() && !args.useTestData) args.useTestData = true;
  return args;
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char* argv[]) {
  const Args args = parseArgs(argc, argv);

  SmootherConfig cfg;
  cfg.lagSeconds              = args.lagSeconds;
  cfg.maxDeadReckoningSeconds = args.maxDeadReckoning;

  // --- Load data ---
  RobotDataset dataset;
  if (args.useTestData) {
    std::cout << "Mode: 5-sample synthetic test dataset\n";
    dataset = generateTestDataset();
  } else {
    std::cout << "Loading CSV: " << args.csvPath << "\n";
    dataset = loadDatasetFromCSV(args.csvPath);
  }

  std::cout << "IMU samples:    " << dataset.imuSamples.size()    << "\n"
            << "Contact events: " << dataset.contactEvents.size() << "\n";

  // --- Estimate initial IMU bias ---
  const InitialBiasEstimate biasEst = estimateInitialBias(dataset, cfg);
  std::cout << "Initial bias (" << biasEst.source
            << ", n=" << biasEst.sampleCount << ")\n"
            << "  acc:  " << biasEst.bias.accelerometer().transpose() << "\n"
            << "  gyro: " << biasEst.bias.gyroscope().transpose()     << "\n";

  // --- Build contact replay plan ---
  const ContactReplayPlan replayPlan =
      buildContactReplayPlan(dataset, cfg.maxDeadReckoningSeconds);
  std::cout << "Contact schedule: "
            << replayPlan.scheduledPacketIndices.size() << " / "
            << replayPlan.packetDecisions.size()        << " packets used\n";

  // --- Prepare output ---
  fs::create_directories(args.outputDir);
  const fs::path trajPath = args.outputDir / "trajectory.csv";
  std::ofstream trajOut(trajPath);
  if (!trajOut)
    throw std::runtime_error("Cannot open output: " + trajPath.string());
  writeTrajectoryHeader(trajOut);

  // --- Run the smoother ---
  const SmootherOutputs outputs = runSmoother(
      dataset, replayPlan, cfg, biasEst.bias, trajOut, args.maxDurationSeconds);

  // --- Summary ---
  std::cout << "\n=== Results ===\n"
            << "Wall time:       " << outputs.wallTimeMs  << " ms\n"
            << "IMU steps:       " << outputs.imuSteps    << "\n"
            << "Contact updates: " << outputs.contactEvents << "\n"
            << "Path length:     " << outputs.pathLength  << " m\n"
            << "Final position:  " << outputs.finalNavState.position().transpose()  << "\n"
            << "Final velocity:  " << outputs.finalNavState.velocity().transpose()  << "\n"
            << "Final bias acc:  " << outputs.finalBias.accelerometer().transpose() << "\n"
            << "Final bias gyro: " << outputs.finalBias.gyroscope().transpose()     << "\n"
            << "Trajectory:      " << trajPath.string() << "\n";

  return 0;
}
