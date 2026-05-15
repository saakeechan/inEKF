clc;
close all;
clear;

%% Load data
T = readtable('/home/saakee/Research/Saaketh_INEKF/Data/G1LoggingData.csv');

%% Parameters
tname = 'timestamp';
n = min(height(T), 500);

time = T.(tname)(1:n);

% cols_left = { ...
%     'joint_pos_left_hip_roll_joint', ...
%     'joint_pos_left_hip_yaw_joint', ...
%     'joint_pos_left_hip_pitch_joint', ...
%     'joint_pos_left_knee_joint', ...
%     'joint_pos_left_ankle_pitch_joint', ...
%     'joint_pos_left_ankle_roll_joint' };

cols_left = { ...
    'joint_vel_left_hip_roll_joint', ...
    'joint_vel_left_hip_yaw_joint', ...
    'joint_vel_left_hip_pitch_joint', ...
    'joint_vel_left_knee_joint', ...
    'joint_vel_left_ankle_pitch_joint', ...
    'joint_vel_left_ankle_roll_joint' };

% cols_left = { ...
%     'joint_torque_left_hip_roll_joint', ...
%     'joint_torque_left_hip_yaw_joint', ...
%     'joint_torque_left_hip_pitch_joint', ...
%     'joint_torque_left_knee_joint', ...
%     'joint_torque_left_ankle_pitch_joint', ...
%     'joint_torque_left_ankle_roll_joint' };

%% Ground-truth contacts
l_gt = logical(T.lfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);

%% Loop through all position columns and plot
for j = 1:numel(cols_left)

    dataName = cols_left{j};
    signal = abs(T.(dataName)(1:n));

    figure;
    hold on;
    grid on;

    % Plot signal first to get y limits
    plot(time, signal, 'b', 'DisplayName', dataName);

    y_limits = ylim;

    % Plot contact regions in red, hidden from legend
    for k = 1:length(l_starts)
        x1 = time(l_starts(k));
        x2 = time(l_ends(k));

        h = patch( ...
            [x1 x2 x2 x1], ...
            [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], ...
            [1.0 0.6 0.6], ...
            'FaceAlpha', 0.3, ...
            'EdgeColor', 'none');

        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end

    % Re-plot signal so it stays on top
    plot(time, signal, 'b', 'DisplayName', dataName);

    xlabel('Time [s]');
    ylabel(dataName, 'Interpreter', 'none');

    title([dataName ' with Left Foot Contact Regions'], ...
          'Interpreter', 'none');

    legend;
end