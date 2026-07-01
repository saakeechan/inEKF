%%

clear; close all; clc;

filePath = "/home/saaketh/Research/inEKF/results/2026-07-01_14-52-53.csv"
savePlots = false;

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
finish = (result1.time(end) / dt);
start = (0 / dt) + 1;
t = start:1:finish - 1;
t = t .* dt;

% --- Contact point velocity magnitudes ---
d1_v = result1.GT_dv_norm_leg1(1:finish-1);
d2_v = result1.GT_dv_norm_leg2(1:finish-1);
contact_leg1 = result1.GT_Contact_leg1(start:finish-1);
contact_leg2 = result1.GT_Contact_leg2(start:finish-1);


% --- Use CSV time column ---
t_plot = result1.time(start:finish-1);

%% Contact point velocity plots

figure(1);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'Per-Leg Contact Regions and dV Norm', 'NumberTitle', 'off');

leg_data = {
    contact_leg1, d1_v(start:finish-1), 'Left Leg';
    contact_leg2, d2_v(start:finish-1), 'Right Leg'
};

for subplot_idx = 1:2
    subplot(2,1,subplot_idx);
    hold on;
    grid minor;

    contact_mask = logical(leg_data{subplot_idx, 1});
    dv_series = leg_data{subplot_idx, 2};
    title_text = leg_data{subplot_idx, 3};

    y_min = min([0; dv_series(:)]);
    y_max = max(dv_series(:));
    if y_max <= y_min
        y_max = y_min + 1;
    end

    hContact = fill(nan, nan, 'r', ...
                    'FaceAlpha', 0.15, ...
                    'EdgeColor', 'none', ...
                    'DisplayName', 'Contact Region');

    in_contact = false;
    contact_start = 1;
    for k = 1:numel(contact_mask)
        if contact_mask(k) && ~in_contact
            in_contact = true;
            contact_start = k;
        elseif ~contact_mask(k) && in_contact
            fill([t_plot(contact_start) t_plot(k-1) t_plot(k-1) t_plot(contact_start)], ...
                 [y_min y_min y_max y_max], ...
                 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            in_contact = false;
        end
    end

    if in_contact
        fill([t_plot(contact_start) t_plot(end) t_plot(end) t_plot(contact_start)], ...
             [y_min y_min y_max y_max], ...
             'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

    hDv = plot(t_plot, dv_series, 'k', 'LineWidth', 0.8, 'DisplayName', 'dV Norm');
    ylim([y_min y_max]);

    title([title_text ' Contact and dV Norm']);
    xlabel('Time [s]');
    ylabel('Velocity magnitude [m/s]');
    legend([hContact, hDv], 'Location', 'best');
end

sgtitle('Per-Leg Contact Regions and dV Norm');


%% Bias Estimates

figure(2);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'Gyroscope and Accelerometer Bias Estimates', 'NumberTitle', 'off');

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

figure(3);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'True vs Estimated Velocities', 'NumberTitle', 'off');

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

figure(4);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'True vs Estimated Roll-Pitch-Yaw', 'NumberTitle', 'off');

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

figure(5);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'Velocity Estimation Error', 'NumberTitle', 'off');

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


%% Position

figure(6);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'True vs Estimated Positions', 'NumberTitle', 'off');

% --- Px ---
subplot(3,1,1);
plot(t_plot, result1.GT_Px(start:finish-1), 'k', ...
     t_plot, result1.EST_Px(start:finish-1), '-.b', 'LineWidth', 0.2);
title('P_x');
xlabel('Time [s]');
ylabel('Position [m]');
legend('P_x true', 'P_x estimated');
grid minor;

% --- Py ---
subplot(3,1,2);
plot(t_plot, result1.GT_Py(start:finish-1), 'k', ...
     t_plot, result1.EST_Py(start:finish-1), '-.b', 'LineWidth', 0.2);
title('P_y');
xlabel('Time [s]');
ylabel('Position [m]');
legend('P_y true', 'P_y estimated');
grid minor;

% --- Pz ---
subplot(3,1,3);
plot(t_plot, result1.GT_Pz(start:finish-1), 'k', ...
     t_plot, result1.EST_Pz(start:finish-1), '-.b', 'LineWidth', 0.2);
title('P_z');
xlabel('Time [s]');
ylabel('Position [m]');
legend('P_z true', 'P_z estimated');
grid minor;

sgtitle('True vs Estimated Positions');


%% Position Error

figure(7);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'Position Estimation Error', 'NumberTitle', 'off');

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

%% RPY Error

figure(8);
clf;
set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Name', 'RPY Estimation Error', 'NumberTitle', 'off');
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

%% Optionally save all open figures
if savePlots
    outputDir = 'output_plots';
    [~, csvBaseName, ~] = fileparts(filePath);
    saveTimestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    outputSubDir = fullfile(outputDir, sprintf('%s_%s', csvBaseName, saveTimestamp));

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    if ~exist(outputSubDir, 'dir')
        mkdir(outputSubDir);
    end

    figHandles = findall(groot, 'Type', 'figure');
    figHandles = figHandles(isgraphics(figHandles, 'figure'));
    figNumbers = arrayfun(@(h) get(h, 'Number'), figHandles);
    [~, sortIdx] = sort(figNumbers);
    figHandles = figHandles(sortIdx);

    for i = 1:numel(figHandles)
        fig = figHandles(i);
        if ~isgraphics(fig, 'figure')
            continue;
        end

        figNum = get(fig, 'Number');
        figTitle = get(fig, 'Name');
        if isempty(figTitle)
            fileStem = sprintf('figure_%02d', figNum);
        else
            fileStem = char(figTitle);
            fileStem = regexprep(fileStem, '[^a-zA-Z0-9_-]+', '_');
            fileStem = regexprep(fileStem, '_+', '_');
            fileStem = regexprep(fileStem, '^_|_$', '');
            if isempty(fileStem)
                fileStem = sprintf('figure_%02d', figNum);
            end
        end

        set(fig, 'Renderer', 'opengl');
        set(fig, 'Units', 'pixels');
        pos = get(fig, 'Position');
        set(fig, 'PaperPositionMode', 'auto', 'Position', pos);
        drawnow;

        outputFilePath = fullfile(outputSubDir, sprintf('%s.png', fileStem));
        print(fig, outputFilePath, '-dpng', '-r300');

        fprintf('Saved: %s\n', outputFilePath);
    end

    fprintf('\nAll open figures saved in: %s\n', outputSubDir);
end
