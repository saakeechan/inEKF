# Estimator mathematical and frame review

Review date: 2026-10-08.

Reviewed files:

- [InEKF_manager.cpp](../src/estimatorManager/InEKF_manager.cpp)
- [RobotModel.cpp](../src/utility/RobotModel.cpp)
- [InEKF.cpp](../src/InEKF/InEKF.cpp)

This records the complete source review, including mathematical issues, frame inconsistencies, conversions, approximations, validation gaps, and redundancies. Line numbers refer to the reviewed snapshot and may move as code changes.

The original review changed no repository files. Subsequently, finding 3 was fixed: live initialization now uses the supplied velocity and biases. The duplicated covariance construction was also resolved in a subsequent cleanup. The other numbered findings below remain review findings, not implemented fixes.

## Must fix

### 1. New contacts use a stale rotation

Location: `InEKF.cpp`, `CorrectKinematics()`, around line 653. The same pattern exists in `CorrectLandmarks()`, around line 418.

`CorrectKinematics()` caches rotation before correction. After correcting existing contacts, it initializes new contacts with:

```text
corrected_position + old_rotation * foot_position
```

This combines state values from two different moments. The new contact covariance also uses the old rotation.

A compiled reproduction with a valid positive-definite covariance initialized a new foot while another foot corrected attitude. The new contact differed by approximately **9.95 cm** from initialization using the corrected state. This is a constructed example, not a measured robot error; the magnitude depends on the attitude correction and foot offset.

Smallest fix: refresh rotation after correction and before augmentation, for both the mean and noise transformation. Apply the same correction to landmark augmentation.

### 2. Quaternion conversion can produce a matrix outside SO(3)

Locations: `InEKF_manager.cpp`, CSV initialization around line 214 and `ReadCSVIMU()` around line 798; conversion helper in `BasicFunctions_Estimator.hpp`.

The callers do not normalize their quaternions, and the helper assumes unit norm. Because its formula is homogeneous in quaternion components, a scaled quaternion produces a scaled rotation matrix rather than a proper rotation.

The configured input CSV contains rounded, slightly non-unit quaternions. The largest observed Frobenius error in `R.transpose() * R - I` was approximately **3.66e-5**. Although small in that dataset, propagation and invariant correction assume valid rotations.

Smallest fix: reject nonfinite or zero-norm quaternions and normalize before conversion. This applies to input CSV quaternions and configured initial orientation/offset quaternions.

### 3. Live initialization ignores configured velocity and biases — fixed

Location: `InEKF_manager.cpp`, live branch of `Initialize()`, around line 227.

At review time, live initialization always set velocity, gyro bias, and accelerometer bias to zero even though `initial_condition_arg` contained them. For example, supplying a calibrated gyro bias still initialized the filter with zero bias.

The follow-up fix uses these segments directly:

| Initial quantity | Zero-based indices in `initial_condition_arg` |
| --- | --- |
| Position | 0–2 |
| Quaternion, wxyz | 3–6 |
| Velocity | 7–9 |
| Gyro bias | 10–12 |
| Accelerometer bias | 13–15 |

The configured velocity and biases now populate the filter, state history, and `robot.state` during live initialization and reinitialization. Regression assertions were added to `tests/imu_timing_test.cpp`.

## Must fix before using the affected modes or frames

### 4. NN velocity correction is missing an attitude Jacobian term

Location: `InEKF.cpp`, `CorrectKinematics()` velocity branch, around line 520.

The velocity residual is:

```text
r = v_F_world - [v + R * ((omega - b_g) × p_IF + p_dot_IF)]
```

Here, `R` maps IMU coordinates to world coordinates, `v` is IMU-origin world velocity, `p_IF` is the IMU-to-foot position expressed in IMU coordinates, and `p_dot_IF` is its relative derivative.

With a zero world foot-velocity target, the zero attitude block is consistent with the invariant formulation. With a nonzero NN target, that cancellation no longer holds.

At a consistent state, under the code's left state perturbation and `H = -dr/delta` convention, the missing attitude block is:

```text
H_theta = -skew(v_F_world)
```

A numerical perturbation with target `(1, 0, 0)` produced attitude sensitivity of magnitude 1; the implementation models that sensitivity as zero.

Consequence: NN correction can allocate errors incorrectly between attitude and velocity. This affects NN-enabled `vkm`; the configured `pkm` path does not execute this branch.

Smallest fix: include the attitude term for the nonzero world velocity target and verify the complete Jacobian using finite differences under the actual state-update convention.

### 5. Relative velocity and Jacobian are incomplete for a moving IMU frame

Locations: `RobotModel.cpp`, `get_foot_velocity()` around line 397 and `get_foot_jacobian()` around line 363.

Foot position is expressed in IMU coordinates. The current velocity calculation is:

```text
R_BI.transpose() * (v_F_B - v_I_B)
```

The derivative of the relative position also requires subtraction of the rotating-frame term:

```text
p_dot_IF = R_BI.transpose() * (v_F_B - v_I_B)
           - omega_I_I × p_IF
```

For the position Jacobian, the corresponding expression is:

```text
J_p_IF = R_BI.transpose()
         * [J_v_F - J_v_I + skew(p_F_B - p_I_B) * J_omega_I]
```

The current Jacobian only subtracts linear parts. It does not rotate into IMU coordinates or include the IMU angular Jacobian.

Pinocchio's `LOCAL_WORLD_ALIGNED` quantities use each frame's origin with axes aligned to the Pinocchio world. They are not automatically expressed in IMU axes. See the [official frame API documentation](https://raw.githubusercontent.com/stack-of-tasks/pinocchio/master/include/pinocchio/algorithm/frames.hpp).

Consequence: an articulated torso IMU can double-count rotational foot motion in the manager's velocity formula and produce incorrectly oriented measurement covariance. The missing terms vanish for a fixed, aligned root IMU, which is the current intended pelvis-IMU configuration.

Smallest fix: make velocity and Jacobian derivatives of the same IMU-relative position returned by `get_foot_position()`.

### 6. Base-velocity transformation misses the lever-arm term

Location: `RobotModel.cpp`, `transform_state_to_base()`, around line 581.

Moving velocity between IMU and base origins requires subtracting:

```text
omega_B_world × (R_WB * p_BI)
```

in addition to the joint-induced relative linear velocity. For a stationary base origin with a rotating body and offset IMU, the IMU has nonzero linear velocity. The current function incorrectly retains that velocity as base translation.

The implementation also treats the Pinocchio universe origin as the robot base. That identity should be established explicitly rather than assumed from frame 0; a model can have an offset root placement.

Smallest fix: identify the actual base frame and include the lever-arm term when converting velocity. The call to this helper is currently commented out in `send_states()`, so it does not affect active output.

## Should fix or explicitly accept as approximations

### 7. Process-noise discretization remains approximate

Location: `InEKF.cpp`, `Propagate()`, around line 215.

The exact exponential of the frozen linearized dynamics is used for the transition:

```text
Phi = exp(A * dt)
```

But the noise calculation remains:

```text
Q_d = PhiAdj * Q * PhiAdj.transpose() * dt
```

This does not evaluate the continuous-noise integral over the interval. An exact `Phi` does not make `Q_d` exact.

For white acceleration noise with unit spectral density and `dt = 0.1`, a compiled check of the actual implementation gave:

| Quantity | Current implementation | Exact continuous-noise integral |
| --- | --- | --- |
| Position variance | 0.001 | 0.000333333 |
| Velocity–position covariance | 0.01 | 0.005 |

This is an inherited approximation; the [upstream implementation](https://raw.githubusercontent.com/RossHartley/invariant-ekf/master/src/InEKF.cpp) explicitly describes its discretized noise as approximate.

Recommendation: use integrated discrete process noise if accurate covariance propagation is required. Treat the current expression as an approximation rather than claiming exact discretization.

### 8. Measurement covariance discards important structure

Location: `InEKF_manager.cpp`, kinematic measurement construction, around line 528. The initialization branch uses the same position covariance approach.

For independent encoders with common position variance, the full transformed covariance is:

```text
Sigma_p = cov_enc_const * J * J.transpose()
```

The implementation keeps only its diagonal through `jacobian.rowwise().squaredNorm()`. This discards correlations between Cartesian axes. It is a modeling approximation, not the full encoder uncertainty transformation.

Separately, ordinary `vkm` uses `cov_enc_const * I` as velocity covariance. Encoder-position variance is not directly foot-velocity uncertainty. Joint-velocity and gyro noise should be mapped into the velocity measurement, with units and frame conventions established explicitly.

Recommendation: preserve the full position covariance when needed, and derive a separate velocity measurement covariance. Combine this with the relative-Jacobian frame correction in finding 5.

### 9. Invalid values can reach the mathematical operations unchecked

- **Time interval:** `Initialize()` does not reject nonfinite, zero, or negative `dt`. Those values invalidate integration and can cause division by zero in ground-truth DV logging.
- **Contact covariance ordering:** `Variable_Contact_Cov()` does not require `cov_slip_const >= cov_contact_const`. A smaller slip covariance makes supposed noise inflation reduce noise instead.
- **NN means:** inferred velocity means are not checked for finiteness before correction or the DV overwrite. A valid variance does not protect against a NaN mean.
- **Configuration failure — resolved:** The manager now checks the return value of `RobotModel::load_config()` and throws with the configuration path on failure, before computing indices or initializing the estimator.
- **Frame IDs:** configured IMU/contact frame IDs are not validated against the loaded model. Contact-frame count should also agree with `num_legs`.

Smallest fixes: validate these values at initialization/inference/model-loading boundaries and stop invalid data from entering the equations. The requested divergence behavior remains printing and continuing; these input validation issues are separate from that policy.

## Redundancies and conversion clarity

- **Duplicated covariance construction — resolved:** The original review found duplicate matrix construction in `SetCovariances()` and `initialize_covariances()`. The follow-up cleanup uses a single `SetCovariances()` function: it reads robot YAML values with explicit built-in defaults and constructs the matrices once during configuration loading. The separate loader, initializer, and constructor covariance assignments were removed.
- **Repeated FK:** NN inference and `Onestep()` both run forward kinematics for the same sample. Any reuse must preserve the independent inference/measurement APIs and ensure the cached sample really matches.
- **Unused norm history — resolved:** Removed the redundant norm-history container, temporary collection vector, append, and reset. DV vectors and norms continue to be saved through `SAVE_MATRIX`.
- **Inactive code:** a commented alternative foot-velocity implementation and unused debug helpers add substantial code that is not executed.
- **Orientation offset comment:** `R_offset * R_csv` applies the offset on the world side, despite the comment saying body frame. A body-side offset would multiply on the right. The correct order depends on the intended offset convention; the comment and contract should agree.

## Checks that did not reveal a sign error

The review did not find a sign error in:

- The standard `pkm` contact Jacobian.
- The `vkm` gyro-bias Jacobian block.
- The Joseph covariance update in `Correct()`.

The previous-IMU propagation/current-measurement correction timing was treated as the existing intended contract. Slip and hard-contact flags were treated as diagnostics; correction intentionally uses nominal `CONTACT_t`.

## Priorities and validation limits

Prioritize findings 1 and 2 for the existing mathematical path. Finding 3 has now been fixed. Resolve finding 4 before relying on NN foot-velocity correction, and findings 5–6 before using the affected moving/offset frame configurations.

The original review compiled the actual filter core with temporary numerical reproductions in `/tmp`. It checked new-contact augmentation, the nonzero NN target's attitude derivative, and acceleration process-noise covariance. It also inspected quaternion norms in the configured input CSV.

The original review did not run full Pinocchio, NN, simulator, or hardware validation. Its numerical examples demonstrate specific mechanisms; they are not claims about observed hardware error or gait performance.
