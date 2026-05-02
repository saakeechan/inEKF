%%

clear; close all; clc;

filePath = "/home/saakee/Research/Saaketh_INEKF/results/2026-05-02_06-25-48.csv"

% --- Read the file ---
if isfile(filePath)
    result1 = readtable(filePath, 'VariableNamingRule', 'preserve');
    disp('File loaded successfully!');
else
    error('File not found:\n%s', filePath);
end


% Basic
[finish, ~] = size(result1);

dt = 0.02;
finish = (19.5 / dt);
start = (0 / dt) + 1;
t = start:1:finish - 1;
t = t .* dt;

slip = 0.3;

% --- Relative velocities and rotation errors ---
v_rel_true = zeros(finish-1, 3);
v_rel_est  = zeros(finish-1, 3);
rot_error  = zeros(finish-1, 3);
phi_error  = zeros(finish-1, 3);
Hrpy_error = zeros(finish-1, 3);

for k = start:finish-1

    % --- Ground-truth rotation from quaternion ---
    q_true = [result1.GT_Qw(k), result1.GT_Qx(k), result1.GT_Qy(k), result1.GT_Qz(k)];
    R_true = quat2rotm(q_true);

    % --- Estimated rotation from quaternion ---
    q_est = [result1.EST_Qw(k), result1.EST_Qx(k), result1.EST_Qy(k), result1.EST_Qz(k)];
    R_est = quat2rotm(q_est);

    % --- Ground-truth and estimated world-frame velocities ---
    v_true = [result1.GT_Vx(k);  result1.GT_Vy(k);  result1.GT_Vz(k)];
    v_est  = [result1.EST_Vx(k); result1.EST_Vy(k); result1.EST_Vz(k)];

    % --- Convert world-frame velocity into body/local frame ---
    v_rel_true(k, 1:3) = (R_true' * v_true)';
    v_rel_est(k, 1:3)  = (R_est'  * v_est)';

    % --- Rotation error metrics ---
    rot_error(k, 1:3)  = rotm2eul(R_true' * R_est);
    phi_error(k, 1:3)  = logm_vec(R_true' * R_est)';
    Hrpy_error(k, 1:3) = [ ...
        result1.EST_roll(k)  - result1.GT_roll(k), ...
        result1.EST_pitch(k) - result1.GT_pitch(k), ...
        result1.EST_yaw(k)   - result1.GT_yaw(k)];
end



% --- Contact point velocity magnitudes ---
d1_v = result1.GT_dv_norm_leg1(1:finish-1);
d2_v = result1.GT_dv_norm_leg2(1:finish-1);


% --- Slip detection for single dataset ---
idx = 0;
slip_pt = [];
bar_height_plus = [];
bar_height_minus = [];

for i = start:finish-1
    if (d1_v(i) > slip && result1.GT_Contact_leg1(i)) || ...
       (d2_v(i) > slip && result1.GT_Contact_leg2(i))

        idx = idx + 1;
        slip_pt(1, idx) = result1.time(i);
        bar_height_plus(1, idx) = 1;
        bar_height_minus(1, idx) = -1;
    end
end


% --- Use CSV time column ---
t_plot = result1.time(start:finish-1);

%% Contact point velocity plot

figure(12);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
hold on;
grid minor;

% --- Plot contact point velocities ---
plot(t_plot, d1_v(start:finish-1), 'r');
plot(t_plot, d2_v(start:finish-1), 'g');

xlabel('Time [s]');
ylabel('Velocity magnitude [m/s]');
title('Contact Point Velocities and Slip Intervals');
legend('Foot 1', 'Foot 2');
ylim auto;

% --- Overlay slip markers as shaded bands ---
yl = ylim;

if ~isempty(slip_pt)
    ax = gca;
    ax.Legend.AutoUpdate = 'off';

    for i = 1:length(slip_pt)
        t_start = slip_pt(i) - 0.01;
        t_end   = slip_pt(i) + 0.01;

        fill([t_start t_end t_end t_start], ...
             [yl(1) yl(1) yl(2) yl(2)], ...
             'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    end

    hSlip = fill([0 0 0 0], [0 0 0 0], 'r', ...
                 'FaceAlpha', 0.15, ...
                 'EdgeColor', 'none', ...
                 'DisplayName', 'Slip Event');

    uistack(hSlip, 'bottom');
    ax.Legend.AutoUpdate = 'on';
end

grid on;
hold off;


%% Relative Velocity Plot

figure(2);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

subplot(3,1,1);
plot(t_plot, v_rel_true(start:finish-1,1), '-k', ...
     t_plot, v_rel_est(start:finish-1,1), '-r', 'LineWidth', 0.2);
title('V_x^{rel}');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_x true', 'V_x estimated');
grid minor;

subplot(3,1,2);
plot(t_plot, v_rel_true(start:finish-1,2), '-k', ...
     t_plot, v_rel_est(start:finish-1,2), '-r', 'LineWidth', 0.2);
title('V_y^{rel}');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_y true', 'V_y estimated');
grid minor;

subplot(3,1,3);
plot(t_plot, v_rel_true(start:finish-1,3), '-k', ...
     t_plot, v_rel_est(start:finish-1,3), '-r', 'LineWidth', 0.2);
title('V_z^{rel}');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_z true', 'V_z estimated');
grid minor;

sgtitle('Relative Velocity Comparison: True vs Estimated');

%% Bias Estimates

figure(1);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% --- Bias X ---
subplot(3,1,1);
plot(t_plot, result1.EST_Bg_x(start:finish-1), 'r', ...
     t_plot, result1.EST_Ba_x(start:finish-1), '-.b', 'LineWidth', 0.2);
title('b_x');
xlabel('Time [s]');
ylabel('Bias');
legend('b_{g,x}', 'b_{a,x}');
grid minor;

% --- Bias Y ---
subplot(3,1,2);
plot(t_plot, result1.EST_Bg_y(start:finish-1), 'r', ...
     t_plot, result1.EST_Ba_y(start:finish-1), '-.b', 'LineWidth', 0.2);
title('b_y');
xlabel('Time [s]');
ylabel('Bias');
legend('b_{g,y}', 'b_{a,y}');
grid minor;

% --- Bias Z ---
subplot(3,1,3);
plot(t_plot, result1.EST_Bg_z(start:finish-1), 'r', ...
     t_plot, result1.EST_Ba_z(start:finish-1), '-.b', 'LineWidth', 0.2);
title('b_z');
xlabel('Time [s]');
ylabel('Bias');
legend('b_{g,z}', 'b_{a,z}');
grid minor;

sgtitle('Gyroscope and Accelerometer Bias Estimates');


%% Velocity

figure(20);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% --- Vx ---
subplot(3,1,1);
plot(t_plot, result1.GT_Vx(start:finish-1), 'k', ...
     t_plot, result1.EST_Vx(start:finish-1), '-.b', 'LineWidth', 0.2);
title('V_x');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_x true', 'V_x estimated');
grid minor;

% --- Vy ---
subplot(3,1,2);
plot(t_plot, result1.GT_Vy(start:finish-1), 'k', ...
     t_plot, result1.EST_Vy(start:finish-1), '-.b', 'LineWidth', 0.2);
title('V_y');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_y true', 'V_y estimated');
grid minor;

% --- Vz ---
subplot(3,1,3);
plot(t_plot, result1.GT_Vz(start:finish-1), 'k', ...
     t_plot, result1.EST_Vz(start:finish-1), '-.b', 'LineWidth', 0.2);
title('V_z');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('V_z true', 'V_z estimated');
grid minor;

sgtitle('True vs Estimated Velocities');


%% RPY

figure(121);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% --- Roll ---
subplot(3,1,1);
plot(t_plot, result1.GT_roll(start:finish-1), 'k', ...
     t_plot, result1.EST_roll(start:finish-1), '-.b', 'LineWidth', 0.2);
title('Roll');
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Roll true', 'Roll estimated');
grid minor;

% --- Pitch ---
subplot(3,1,2);
plot(t_plot, result1.GT_pitch(start:finish-1), 'k', ...
     t_plot, result1.EST_pitch(start:finish-1), '-.b', 'LineWidth', 0.2);
title('Pitch');
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Pitch true', 'Pitch estimated');
grid minor;

% --- Yaw ---
subplot(3,1,3);
plot(t_plot, result1.GT_yaw(start:finish-1), 'k', ...
     t_plot, result1.EST_yaw(start:finish-1), '-.b', 'LineWidth', 0.2);
title('Yaw');
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Yaw true', 'Yaw estimated');
grid minor;

sgtitle('True vs Estimated Roll-Pitch-Yaw');


%% Velocity Error

figure(4);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% --- Vx Error ---
subplot(3,1,1);
vx_error = result1.EST_Vx(start:finish-1) - result1.GT_Vx(start:finish-1);
plot(t_plot, vx_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('V_x Error');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
rmse_vx = rms(vx_error);
legend(['RMSE: ' num2str(rmse_vx, '%.4f')]);

% --- Vy Error ---
subplot(3,1,2);
vy_error = result1.EST_Vy(start:finish-1) - result1.GT_Vy(start:finish-1);
plot(t_plot, vy_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('V_y Error');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
rmse_vy = rms(vy_error);
legend(['RMSE: ' num2str(rmse_vy, '%.4f')]);

% --- Vz Error ---
subplot(3,1,3);
vz_error = result1.EST_Vz(start:finish-1) - result1.GT_Vz(start:finish-1);
plot(t_plot, vz_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('V_z Error');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
rmse_vz = rms(vz_error);
legend(['RMSE: ' num2str(rmse_vz, '%.4f')]);

sgtitle('Velocity Estimation Error');


%% Position Error

figure(5);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

% --- Px Error ---
subplot(3,1,1);
px_error = result1.EST_Px(start:finish-1) - result1.GT_Px(start:finish-1);
plot(t_plot, px_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('P_x Error');
xlabel('Time [s]');
ylabel('Position [m]');
rmse_px = rms(px_error);
legend(['RMSE: ' num2str(rmse_px, '%.4f')]);

% --- Py Error ---
subplot(3,1,2);
py_error = result1.EST_Py(start:finish-1) - result1.GT_Py(start:finish-1);
plot(t_plot, py_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('P_y Error');
xlabel('Time [s]');
ylabel('Position [m]');
rmse_py = rms(py_error);
legend(['RMSE: ' num2str(rmse_py, '%.4f')]);

% --- Pz Error ---
subplot(3,1,3);
pz_error = result1.EST_Pz(start:finish-1) - result1.GT_Pz(start:finish-1);
plot(t_plot, pz_error, 'b','LineWidth', 0.2);
yline(0, '--k');
grid minor;
title('P_z Error');
xlabel('Time [s]');
ylabel('Position [m]');
rmse_pz = rms(pz_error);
legend(['RMSE: ' num2str(rmse_pz, '%.4f')]);

sgtitle('Position Estimation Error');

%% Relative Velocity Error

figure(6);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

rel_labels = {'V_x^{rel}', 'V_y^{rel}', 'V_z^{rel}'};

for j = 1:3
    subplot(3,1,j);

    v_rel_error = v_rel_est(start:finish-1,j) - v_rel_true(start:finish-1,j);

    plot(t_plot, v_rel_error, 'b', 'LineWidth', 0.2);
    yline(0, '--k');
    grid minor;

    title(['Relative ' rel_labels{j} ' Error']);
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');

    rmse_rel = rms(v_rel_error);
    legend(['RMSE: ' num2str(rmse_rel, '%.4f')]);

    hold on;
    if exist('slip_pt', 'var') && ~isempty(slip_pt)
        bar(slip_pt, bar_height_plus, 1, ...
            'FaceAlpha', 0.2, ...
            'FaceColor', [1 0 0], ...
            'LineStyle', 'none');

        bar(slip_pt, bar_height_minus, 1, ...
            'FaceAlpha', 0.2, ...
            'FaceColor', [1 0 0], ...
            'LineStyle', 'none');
    end
end

sgtitle('Relative Velocity Error');

%% RPY Error

figure(31);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
sgtitle('RPY Estimation Error');

labels = {'Roll', 'Pitch', 'Yaw'};

for j = 1:3
    subplot(3,1,j);

    if j == 1
        err = result1.EST_roll(start:finish-1) - result1.GT_roll(start:finish-1);
    elseif j == 2
        err = result1.EST_pitch(start:finish-1) - result1.GT_pitch(start:finish-1);
    else
        err = result1.EST_yaw(start:finish-1) - result1.GT_yaw(start:finish-1);
    end

    plot(t_plot, err, 'b','LineWidth', 0.2);
    yline(0, '--k');
    grid minor;

    title([labels{j} ' Error']);
    xlabel('Time [s]');
    ylabel('Angle [rad]');

    rmse_val = rms(err);
    legend(['RMSE: ' num2str(rmse_val, '%.4f')]);

    hold on;
    if ~isempty(slip_pt)
        bar(slip_pt, bar_height_plus, 1, 'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
        bar(slip_pt, bar_height_minus,1,'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
    end
end


%% Log-map Error

figure(33);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
sgtitle('Log-Map Orientation Error');

labels = {'Roll', 'Pitch', 'Yaw'};

for j = 1:3
    subplot(3,1,j);

    err = phi_error(start:finish-1,j);

    plot(t_plot, err, 'b','LineWidth', 0.2);
    yline(0, '--k');
    grid minor;

    title(['logm Error ' labels{j}]);
    xlabel('Time [s]');
    ylabel('Angle [rad]');

    rmse_val = rms(err);
    legend(['RMSE: ' num2str(rmse_val, '%.4f')]);

    hold on;
    if ~isempty(slip_pt)
        bar(slip_pt, bar_height_plus, 1, 'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
        bar(slip_pt, bar_height_minus,1,'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
    end
end


%% H-RPY Error

figure(34);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
sgtitle('H RPY Orientation Error');

labels = {'Roll', 'Pitch', 'Yaw'};

for j = 1:3
    subplot(3,1,j);

    err = Hrpy_error(start:finish-1,j);

    plot(t_plot, err, 'b','LineWidth', 0.2);
    yline(0, '--k');
    grid minor;

    title(['H RPY Error ' labels{j}]);
    xlabel('Time [s]');
    ylabel('Angle [rad]');

    rmse_val = rms(err);
    legend(['RMSE: ' num2str(rmse_val, '%.4f')]);

    hold on;
    if ~isempty(slip_pt)
        bar(slip_pt, bar_height_plus, 1, 'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
        bar(slip_pt, bar_height_minus,1,'FaceAlpha',0.2,'FaceColor',[1 0 0],'LineStyle','none');
    end
end


%%

function eul = Rotation_to_Euler(R)
%ROTATION_TO_EULER Convert a rotation matrix to ZYX Euler angles.
%   eul = [roll, pitch, yaw] in radians.
%
%   The convention used is:
%       R = Rz(yaw) * Ry(pitch) * Rx(roll)
%
%   Inputs:
%       R - 3x3 rotation matrix (SO(3))
%   Outputs:
%       eul - [roll, pitch, yaw] (1x3) in radians

    % Ensure numerical orthogonality
    R = R ./ norm(R(:,1)); % normalize columns collectively if scaled

    % Safeguard for rounding errors in R(3,1)
    r31 = max(min(-R(3,1), 1), -1);
    pitch = asin(r31);

    if abs(cos(pitch)) > 1e-6
        roll = atan2(R(3,2), R(3,3));
        yaw  = atan2(R(2,1), R(1,1));
    else
        % Gimbal lock: pitch ≈ ±pi/2
        roll = 0;
        yaw  = atan2(-R(1,2), R(2,2));
    end

    eul = [roll, pitch, yaw];
end

function phi = logm_vec(R)
%LOGM_VEC Convert rotation matrix R ∈ SO(3) to its logarithm (rotation vector).
%   phi = logm_vec(R) returns a 3x1 vector such that expm(skew(phi)) = R.

    % Numerical guard for det(R)
    d = det(R);
    if abs(d - 1) > 1e-3
        warning('logm_vec:InvalidSO3', 'Matrix determinant deviates from 1 (det = %.4f)', d);
        R = R / nthroot(d, 3); % normalize if slightly off SO(3)
    end

    % Clamp trace to valid range
    tr = max(min(trace(R), 3), -1);
    theta = acos((tr - 1) / 2);

    if abs(theta) < 1e-12
        % First-order approximation for small angles
        phi = 0.5 * [R(3,2) - R(2,3);
                     R(1,3) - R(3,1);
                     R(2,1) - R(1,2)];
    else
        phi = (theta / (2*sin(theta))) * [R(3,2) - R(2,3);
                                          R(1,3) - R(3,1);
                                          R(2,1) - R(1,2)];
    end
end

%% Save all open figures into a specific folder (preserving appearance)
% Define base output directory
outputDir = 'output_images';

% Create base folder if it doesn't exist
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Define subfolder name using fileName(1:end-4)
outputSubDir = fullfile(outputDir, [fileName(1:end-4) '_pictures']);

% Create subfolder if it doesn't exist
if ~exist(outputSubDir, 'dir')
    mkdir(outputSubDir);
end

% Find all open figure handles and sort by figure number
figHandles = findobj('Type', 'figure');
figHandles = sort(figHandles);

% Loop through and export each figure
for i = 1:numel(figHandles)
    fig = figHandles(i);
    figNum = fig.Number;

    % Match on-screen appearance
    set(fig, 'Renderer', 'opengl');
    set(fig, 'Units', 'pixels');
    pos = get(fig, 'Position');
    set(fig, 'PaperPositionMode', 'auto', 'Position', pos);

    % Construct file path
    filePath = fullfile(outputSubDir, sprintf('figure_%02d.png', figNum));

    % Export with consistent resolution and true color fidelity
    exportgraphics(fig, filePath, 'Resolution', 300, 'BackgroundColor', 'none');

    fprintf('Saved: %s\n', filePath);
end

fprintf('\nAll open figures saved in: %s\n', outputSubDir);

close all;