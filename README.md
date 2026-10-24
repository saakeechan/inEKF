# inEKF

C++ invariant extended Kalman filtering for legged robot state estimation, with Pinocchio robot kinematics, YAML configuration, CSV replay, and a shared library for integration into another application.

The estimator propagates IMU measurements and can apply contact kinematics, an external body velocity measurement, or both. The included replay application uses a G1 CSV reader. You can use your own reader or live sensor adapter by producing the same measurement variables.

## Contents

- [Build](#build)
- [Extract robot parameters and configure joint mappings](#extract-robot-parameters-and-configure-joint-mappings)
- [Configure the estimator](#configure-the-estimator)
- [Replay a CSV](#replay-a-csv)
- [Use Onestep in your application](#use-onestep-in-your-application)
- [Write your own CSV reader or sensor adapter](#write-your-own-csv-reader-or-sensor-adapter)
- [Read estimated states](#read-estimated-states)
- [Other tools and source layout](#other-tools-and-source-layout)
- [Planned features](#planned-features)

## Build

Run the following host commands from the repository root. Docker provides the compiler and dependencies used by the project:

```bash
docker build -f docker/Dockerfile -t inekf .
bash docker/docker.sh run
```

Inside the container, the repository is mounted at `/workspace`:

```bash
cd /workspace
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DARCH=x64
cmake --build build --target INEKF_Estimator INEKF_Estimator_Test -j2
```

This builds:

- `build/libINEKF_Estimator_x64.so`: shared estimator library.
- `build/INEKF_Estimator_Test`: CSV replay application from [src/main.cpp](src/main.cpp).

The x64 Docker build command explicitly selects `docker/Dockerfile`; the helper's `build` subcommand currently looks for a Dockerfile in the working directory.

For a native build, provide a C++17 compiler, CMake, Eigen3, Pinocchio with URDF/MJCF parsers and pkg-config metadata, yaml-cpp, Boost, urdfdom, console_bridge, and ONNX Runtime. See [CMakeLists.txt](CMakeLists.txt) for the exact linking requirements. The current x64 build expects ONNX Runtime headers directly in `/usr/local/include` and `libonnxruntime.so.1.25.1` in `/usr/local/lib`, including when NN inference is disabled.

For ARM64/Jetson, the repository also provides:

```bash
bash docker/docker.sh build-arm64
bash docker/docker.sh run-arm64
```

Then, inside that container:

```bash
cd /workspace
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DARCH=arm64
cmake --build build --target INEKF_Estimator INEKF_Estimator_Test -j2
```

The ARM64 helper mounts a separate host `build-arm64/` directory at `/workspace/build`. It produces `libINEKF_Estimator_arm64.so` and expects ONNX Runtime 1.24.2 at the architecture-specific path in CMake. Building an ARM64 image on an x64 host requires working Docker Buildx/emulation support.

## Extract robot parameters and configure joint mappings

Do this when adding a robot or changing its URDF/MJCF model. The report tells you which configuration indices, velocity indices, and frame IDs belong to each joint and frame. It does not automatically generate the robot YAML.

### 1. Run getParameters on the exact model you will use

Temporarily put this at the beginning of `main()` in [src/main.cpp](src/main.cpp), before constructing the estimator:

```cpp
INEKFilter_manager::getParameters(
    "../src/utility/G1/g1_29dof_wFootFrames.xml", "G1");
return 0;
```

Rebuild, then run from the build directory:

```bash
cmake --build build --target INEKF_Estimator_Test -j2
cd build
./INEKF_Estimator_Test
```

This example writes `../G1Parameters`, which is `G1Parameters` at the repository root when run from `build/`. The general output name is `../<robot_name>Parameters`, relative to the process working directory. Supported input extensions are `.urdf`, `.xml`, and `.mjcf`.

The report contains:

- Model dimensions `nq` and `nv`.
- Joint names and their configuration (`q`) and velocity (`v`) indices.
- Per-joint configuration and velocity dimensions.
- Frame names, frame indices, and parent joints.

After generating the report, remove the temporary call and `return 0;`, then rebuild to restore normal replay. There is currently no separate `getParameters` executable or command-line switch.

### 2. Create the robot YAML

Copy [config/G1_config.yaml](config/G1_config.yaml) as a starting point, then set:

| Setting | Meaning |
| --- | --- |
| `robot_name` | Robot identifier. The existing CSV IMU adapter has special handling for the exact name `G1`. |
| `model.model_path` | Path to the exact model used for parameter extraction. |
| `model.format` | `urdf` or `mjcf`. |
| `structure.num_legs` | Number of modeled contact points. |
| `structure.contact_frame_ids` | One Pinocchio frame ID per contact, in your chosen contact order. |
| `imuFrame_id` | Pinocchio frame receiving the IMU measurements. |
| `stateFrame_id` | Pinocchio frame whose estimated pose and velocity you want as output. Defaults to the IMU frame if omitted. |
| `Encoder_indices_map` | Destination `q` index for each incoming joint position. |
| `Encoderdot_indices_map` | Destination `v` index for each incoming joint velocity. |
| `Leg_indices_map` | Indices into incoming joint/torque arrays, grouped by leg, for model-based contact detection. |
| `covariance` | Robot covariance defaults. |

Use the generated report to choose frame IDs; IDs can change when the model changes. Confirm that the selected frames represent the intended physical contact points and IMU location.

### 3. Map sensor order to model order

Mapping entries are **destination indices**, not indices used to reorder the incoming array:

```text
q[Encoder_indices_map[i]]       = incoming_joint_position[i]
v[Encoderdot_indices_map[i]]    = incoming_joint_velocity[i]
```

For example, if incoming joints are `[joint_B, joint_A]`, but the parameter report places `joint_A` at index 0 and `joint_B` at index 1 in both vectors:

```yaml
Encoder_indices_map: [1, 0]
Encoderdot_indices_map: [1, 0]
```

Each incoming motor frame must contain exactly as many joints as each mapping list. `Onestep` accepts a rectangular `2 x N` matrix, so both lists must have length `N`. Position and velocity destination indices can differ: `nq` and `nv` are not always equal.

The included G1 reader emits 29 joints in this order:

| Incoming indices | Joint order |
| --- | --- |
| 0–5 | Left hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll |
| 6–11 | Right hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll |
| 12–14 | Waist yaw, roll, pitch |
| 15–21 | Left shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw |
| 22–28 | Right shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw |

The supplied G1 YAML maps these to indices 0–28. For another model, derive the mapping from its report rather than assuming this identity mapping.

Unmapped `q` and `v` entries are currently filled with zero. A floating-base or multi-coordinate joint model may need additional valid configuration values, such as a unit quaternion; joint index lists alone do not initialize those values. Check the model representation and adapt the mapping before using such a model. [RobotModel.cpp](src/utility/RobotModel.cpp) contains the implementation.

## Configure the estimator

The constructor takes **initialization YAML first, robot YAML second**:

```cpp
INEKFilter_manager estimator(
    "../config/initialization_config.yaml",
    "../config/G1_config.yaml");
```

Paths in YAML and in the application are resolved relative to the process working directory. The supplied paths assume you run the executable from `build/`.

Use [config/initialization_config.yaml](config/initialization_config.yaml) to configure:

| Setting | Usage |
| --- | --- |
| `dt` | Propagation interval in seconds. Set it to the actual interval between IMU samples. |
| `sliding_window_size` | History buffer length, at least 2. Use 2 for the ordinary filter. Larger buffers do not enable smoothing. |
| `kinematics_mode` | Use `pkm` for the current position kinematics correction. A `vkm` branch exists but is experimental and marked broken in the supplied config. |
| `textfile.enabled` | `true` for the built-in CSV replay; `false` when your caller supplies measurements. |
| `textfile.file_name` | Input CSV path. |
| `textfile.start_count` | Zero-based data row used for initialization, excluding the header. |
| `textfile.end_count` | Inclusive final data row. Values beyond the file are capped. |
| `textfile.save_results` | Whether the replay executable writes results at the end. |
| `textfile.saved_results_folder` | Output directory for result CSVs and metadata. |
| `Contact_estimation_NN.enabled` | Loads the contact NN in live mode; callers must explicitly invoke inference. CSV replay uses recorded contacts. |
| `slip_rejection` | Enables slip diagnostics and raises contact process noise to the slip value when the preceding residual exceeds the threshold. |
| `variable_contact_covariance` | Scales contact process noise using the preceding foot velocity residual, capped at the slip value. Takes precedence over threshold-based noise switching when both options are enabled. |
| `covariance_exponents` | Overrides robot covariance defaults using `10^value`. For example, `encoder: -8` selects `1e-8`. |
| `initial_condition` | Position, quaternion `[w,x,y,z]`, velocity, gyro bias, and accelerometer bias. |

With CSV replay enabled, initial position and velocity are taken from the selected CSV row plus the YAML offsets. Initial orientation is `R_offset * R_csv`. Biases come from the YAML. With CSV replay disabled, the YAML provides the initial state directly; `Onestep` does not infer initial orientation from an IMU quaternion.

Slip handling adjusts uncertainty; it does not remove a contact simply because `Hard_Contact` becomes false. The kinematic correction uses the supplied `contacts` flags.

## Replay a CSV

1. Configure your robot and joint mappings.
2. Set `textfile.enabled: true`, choose a CSV file, and set the row range in the initialization YAML.
3. Keep `kinematics_mode: pkm` for the current replay path.
4. Build and run from `build/`:

```bash
cmake --build build --target INEKF_Estimator_Test -j2
cd build
./INEKF_Estimator_Test
```

The executable has no command-line arguments. Change the constructor paths in `src/main.cpp` if you use different YAML filenames. The configured example recording is local data and is not supplied by a fresh clone.

The replay loop is equivalent to:

```cpp
INEKFilter_manager estimator(
    "../config/initialization_config.yaml", "../config/G1_config.yaml");

for (int row = estimator.start_index + 1; row <= estimator.time_length; ++row) {
    estimator.RunCSVStep();
    // Latest estimate: estimator.robot.state
}
```

Construction initializes from `start_count`; updates start at `start_count + 1`. `RunCSVStep()` reads the current row, calls `Onestep()` once with recorded contacts and joint data, and records the completed state for export.

Replay uses `dt` from YAML, **not differences between CSV timestamps**. Resample irregular recordings or adapt the caller to supply the correct interval. The CSV contact comparison printed by the example compares against the same recorded contacts used as input; it is not an independent contact detector evaluation.

When saving is enabled, `src/main.cpp` calls `InEKF_SaveUtils::write_saved_data_impl(estimator)` after replay. Results are named `<timestamp>.csv`, with configuration metadata under `results_meta/<timestamp>_info.txt` in the configured output folder. Exported time starts at zero and uses saved-row index times `dt`, rather than the original CSV timestamps. The current logger has a fixed capacity of 140,000 saved rows, including initialization; keep replay within that capacity. Direct `Onestep()` calls do not automatically execute the replay logging/export path.

## Use Onestep in your application

Include the manager header and link the built estimator library:

```cpp
#include "estimatorManager/InEKF_manager.hpp" // Add this repo's src/ to include paths.
```

The public step API is:

```cpp
void Onestep(const Eigen::VectorXd& fresh_imu,
             const StepMeasurements& measurements = {});
```

For a live caller or your own reader, copy the initialization YAML and set `textfile.enabled: false`. Construct the manager with that YAML and your robot YAML. This avoids loading the example CSV or initializing from its ground truth. The provided `main.cpp` is a replay program; disabling CSV requires your own acquisition/step loop.

### Common input variables

Prepare these from one synchronized sensor sample:

```cpp
Eigen::VectorXd imu(6);
// gyro: rad/s; accelerometer: specific force in m/s^2; both in IMU axes.
imu << gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;

Eigen::MatrixXd motor_data(2, number_of_incoming_joints);
motor_data.row(0) = joint_positions.transpose();  // Incoming order, radians.
motor_data.row(1) = joint_velocities.transpose(); // Same order, rad/s.

std::vector<bool> contacts = contact_flags; // One flag per configured contact frame.
```

Names such as `gyro_x`, `joint_positions`, and `contact_flags` above represent data supplied by your adapter. All IMU and joint values must be finite. Motor data must match both mapping list lengths, and contacts must match `structure.num_legs`.

Accelerometer input to `Onestep` must include gravity as specific force: a stationary, level IMU normally reports approximately `[0, 0, +9.80665]`. The filter adds world gravity during propagation. Supply the measurement convention expected by this API rather than applying the built-in G1 CSV conversion to raw hardware data.

### A. IMU only

```cpp
estimator.Onestep(imu);
```

This propagates without a new kinematic or body velocity correction. Existing contact anchors and contact flags are retained; omitting kinematics is not a liftoff message. Foot velocity diagnostics are unavailable (`NaN`) without fresh joint data.

### B. IMU with contact kinematics

```cpp
StepMeasurements measurements;
measurements.kinematics = KinematicsMeasurement{contacts, motor_data};
estimator.Onestep(imu, measurements);
```

This maps joint data into the robot model, updates forward kinematics, propagates, and applies the configured kinematic correction. A supplied `false` contact flag removes that contact anchor; a new `true` flag initializes an anchor.

### C. IMU with external body velocity

```cpp
StepMeasurements measurements;
measurements.body_velocity = BodyVelocityMeasurement{
    velocity_in_imu_axes,        // Eigen::Vector3d, m/s.
    diagonal_velocity_variances  // Eigen::Vector3d, (m/s)^2.
};
estimator.Onestep(imu, measurements);
```

The velocity is the velocity of the **IMU origin, expressed in IMU axes**. A velocity reported at another body origin needs the appropriate frame and lever-arm conversion first. Variances are diagonal in those same axes and must be finite and strictly positive; supply variances, not standard deviations. The filter rotates this covariance into world axes internally. The current body velocity update uses the ordinary correction and can affect attitude and biases through covariance coupling.

### D. IMU with both measurements

```cpp
StepMeasurements measurements;
measurements.kinematics = KinematicsMeasurement{contacts, motor_data};
measurements.body_velocity = BodyVelocityMeasurement{
    velocity_in_imu_axes, diagonal_velocity_variances};
estimator.Onestep(imu, measurements);
```

Choose one of A–D for each IMU timestep. Combining both optional measurements in one call propagates once, corrects kinematics first, then body velocity, updates output, and shifts history once. Calling B and C consecutively for the same timestep would propagate twice. Include an optional measurement only when it is fresh; do not repeatedly fuse a cached lower-rate velocity prediction.

Each step uses the previous buffered IMU sample for propagation and the supplied fresh sample for the current slot. The first live call seeds previous IMU history from the first sample and still propagates by `dt`. The API has no timestamp argument; set the public `estimator.dt` to the appropriate positive interval before a step if your caller manages variable timing.

### Resolve contacts before stepping

`Onestep()` consumes resolved contact flags. It does not automatically run a detector based on the YAML flag. You may use external contacts, or call one of the provided live helpers before the kinematic step:

```cpp
// Model-based detector: latest torque and joint velocity arrays in incoming order.
contacts = estimator.run_contact_detection(motor_torques, joint_velocities);
```

Configure `Leg_indices_map` for that order. This helper requires CSV mode to be disabled.

The NN alternative requires `Contact_estimation_NN.enabled: true` and a compatible model:

```cpp
auto [predicted_contacts, foot_speed_norms] = estimator.run_contact_inference(
    imu, motor_data, motor_torques, motor_position_commands, commanded_velocity);
contacts = predicted_contacts;
```

Pass the resulting `contacts` into `KinematicsMeasurement` for the same step. The contact NN path is experimental and model-specific; verify its feature order, window, units, and output contract before using a different model. Its predicted **foot** velocities are separate from `BodyVelocityMeasurement`, which accepts an external velocity of the IMU origin.

## Write your own CSV reader or sensor adapter

The filter does not require a particular CSV layout or ROS topic name. The common interface is the measurement variables passed to `Onestep`. Your reader can consume differently named columns or topics as long as it maps them into the same units, frames, ordering, and shapes.

### Option 1: Produce Onestep inputs directly

Use your own parser/subscriber, initialize with CSV mode disabled, and populate:

| Common quantity | Required representation |
| --- | --- |
| IMU | Six values `[gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z]` in IMU axes. |
| Joint measurements, for kinematic correction | `2 x N` matrix `[q; qd]` in the incoming order described by the robot YAML. |
| Contacts, for kinematic correction | `num_legs` Boolean flags in `contact_frame_ids` order. |
| Optional external velocity | IMU-origin velocity in IMU axes and three positive diagonal variances. |
| Timing | One step per new IMU sample, with the correct propagation interval. |

This is the simplest route for another dataset or live transport. Pose, ground truth, torques, and commands are not required by `Onestep` itself; torques and commands are needed only if you choose a contact detector that consumes them. Initialize pose and biases through your YAML or initialization code.

### Option 2: Preserve the built-in replay interface

The current reader is [src/utility/csvParser.cpp](src/utility/csvParser.cpp), exposing:

```cpp
DataStruct ParseCSV(const std::string& csv_filename);
```

It matches columns by header name, so column order can change. It parses numeric comma-separated rows; it is not a general quoted-field CSV parser. Missing values often become `NaN`, which does not make required sensor fields optional.

For the included G1 reader, the main replay fields are:

| CSV headers | Shared DataStruct variables / purpose |
| --- | --- |
| `timestamp` | `timestamp`; its vector length determines the replay row count. It does not set `dt`. |
| `gyro_body_x`, `gyro_body_y`, `gyro_body_z` | Body/IMU gyro vectors. |
| `acc_body_x`, `acc_body_y`, `acc_body_z` | Recorded acceleration vectors; see the G1 conversion below. |
| `joint_pos_<joint_name>` | Packed into `encoders[row]` in the documented 29-joint order. Example: `joint_pos_left_hip_pitch_joint`. |
| `joint_vel_<joint_name>` | Packed into `encoderdots[row]` in the same order. Example: `joint_vel_left_hip_pitch_joint`. |
| `lfoot_contact`, `rfoot_contact` | Packed into `contact_state[row]`, left then right. Use explicit finite 0/1 values. |
| `pos_x`, `pos_y`, `pos_z` | Initial IMU-origin position in world axes, and reference logging. |
| `quat_w`, `quat_i`, `quat_j`, `quat_k` | Initial IMU orientation and G1 acceleration conversion; quaternion order is `[w,x,y,z]`. |
| `vel_x`, `vel_y`, `vel_z` | Initial IMU-origin velocity in world axes, and reference logging. |

The reader also extracts torque, command, foot pose, and reference fields. See [csvParser.hpp](src/utility/csvParser.hpp) for the full `DataStruct` and [inekf_save_utils.cpp](src/utility/inekf_save_utils.cpp) for fields consumed by logging. Reference fields omitted from the recording can appear as `NaN` in output; initialization position, orientation, and velocity must be valid.

For `robot_name: G1`, `ReadCSVIMU()` converts the existing recording's acceleration convention using:

```text
specific_force_IMU = recorded_acceleration_IMU - R_IMU_to_world.transpose() * gravity_world
gravity_world = [0, 0, -9.806659]
```

If your CSV already stores raw accelerometer specific force, adapt `ReadCSVIMU()` to avoid adding gravity a second time. This conversion also requires a valid recorded quaternion on every G1 replay row. Other robot names currently bypass it.

To integrate another schema with `RunCSVStep()`, adapt or replace the parser while preserving `DataStruct`, `ParseCSV()`, and the shared fields consumed by initialization, `ReadCSVInputs()`, and logging. Set row counts and dimensions consistently, fill `encoders`, `encoderdots`, and `contact_state` for every row, and update their robot-specific packing. Adjust `ReadCSVIMU()` if the IMU convention differs. Keep a single active implementation of `ParseCSV()` in the estimator target.

Changing the robot YAML alone does not replace the G1 CSV packing. The Digit and MuJoCo readers are separate source adapters, not automatically selected by `robot_name`. The built-in replay always supplies a kinematic measurement using its recorded contacts; use a custom `Onestep` loop if your dataset needs IMU-only steps or sparse external predictions.

## Read estimated states

After `Onestep()` or `RunCSVStep()` returns:

```cpp
const auto& state = estimator.robot.state;
std::cout << "Position: " << state.Position.transpose() << '\n';
std::cout << "Velocity: " << state.Velocity.transpose() << '\n';
```

`Rotation`, `Position`, `Velocity`, and `Angular_Velocity` refer to the configured `stateFrame_id`, expressed in world axes; `Rotation` maps that frame's axes into world axes. `Bias_Gyro` and `Bias_Acc` remain IMU bias quantities. `Covariance` is the raw IMU filter error-state covariance, including active contact states, and is not transformed to `stateFrame_id`.

`Contact`, `Hard_Contact`, `Slip`, `d_v`, and `dv_norm` provide contact diagnostics. `d_v` contains stacked world-frame foot velocity residuals and `dv_norm` their per-contact magnitudes, or NN foot velocity diagnostics when that path supplied them. Residual diagnostics alone do not enable a separate velocity residual correction.

If output is requested at a frame other than the IMU, joint-dependent frame transforms use the latest available forward kinematics. For raw IMU-filter state access, use `estimator.filter.getState()`.

When embedding this repository with CMake, link target `INEKF_Estimator` and make `src/` plus the required Eigen/Pinocchio/ONNX include paths available to your consumer. The output library filename depends on `ARCH`. Current installation rules install headers only; there is no complete installed library/package export, so use the build tree or add library installation rules in your integration.

## Other tools and source layout

| Path / target | Purpose |
| --- | --- |
| [src/InEKF/](src/InEKF/) | Filter state, propagation, correction, Lie group, and noise implementation. |
| [src/estimatorManager/](src/estimatorManager/) | Initialization, unified step API, history, contact diagnostics, and CSV replay orchestration. |
| [src/utility/RobotModel.hpp](src/utility/RobotModel.hpp) | Robot loading, encoder mapping, foot kinematics, and output frame transforms. |
| [src/contactDetection/](src/contactDetection/) | Model-based and ONNX contact detector implementations. |
| [config/](config/) | Robot and estimator YAML files. The Digit example requires an external model path and adapter setup. |
| [tests/](tests/) | Focused regression sources/scripts; these are not currently registered as CMake/CTest targets. |
| [Extra_Files/callerINEKF.cpp](Extra_Files/callerINEKF.cpp) | Illustrative ROS2 caller with placeholders and robot-specific assumptions; not a ready-to-run ROS package. |
| `writeFKFootPosVel` | Appends/replaces FK fields in CSVs selected by a hard-coded file list. |
| `writeFKFootPosVelMujoco` | Appends/replaces FK fields in CSVs under `../MujocoData` using the separate MuJoCo reader. |

Build an optional tool with, for example, `cmake --build build --target writeFKFootPosVel -j2`. Review its input paths before running: the FK writers modify their selected CSV files in place. Normal estimator replay computes FK itself and does not require this preprocessing.

## Planned features

- **Velocity kinematic residual correction:** complete and validate the velocity-based kinematic measurement formulation. The existing experimental `vkm` branch and foot velocity diagnostics are groundwork; `pkm` remains the current usage path.
- **Smoother variants:** add smoothing beyond the present forward filter. Increasing `sliding_window_size` currently changes buffered history only; it does not perform backward or fixed-lag smoothing.
