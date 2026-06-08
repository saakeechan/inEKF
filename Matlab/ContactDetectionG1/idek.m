clc;
close all;
clear;

%% Load data
T = readtable('/home/saakee/Research/Saaketh_INEKF/Data/robotstate_0_env2.csv');

%% Parameters
tname = 'timestamp';
n = min(height(T), 500);

time = T.(tname)(1:n);
time = time(:);

dt = median(diff(time));
fs = 1 / dt;

fprintf('Estimated fs = %.3f Hz\n', fs);

%% Ground-truth contacts
l_gt = logical(T.lfoot_contact(1:n));
l_gt = l_gt(:);

%% Signal: acceleration from velocity finite difference

dataName1 = "fk_left_foot_vel_x";
dataName2 = "fk_left_foot_vel_y";
dataName3 = "fk_left_foot_vel_z";

v1 = T.(dataName1)(1:n);
v2 = T.(dataName2)(1:n);
v3 = T.(dataName3)(1:n);

v1 = v1(:);
v2 = v2(:);
v3 = v3(:);

% Per-timestep 3D velocity magnitude
v = vecnorm([v1], 2, 2);

% Acceleration estimate: a[k] = (v[k] - v[k-1]) / dt
% signal = diff(v) / dt;
signal = v1(2:end);

% diff reduces length by 1, so align time and contact labels
time_signal = time(2:end);
l_gt_signal = l_gt(2:end);

signalName = dataName1 + "_accel";

%% Contact regions after diff alignment

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt_signal);

%% Plot only acceleration inside contact regions

figure;
hold on;
grid on;

% Plot only contact samples
plot(time_signal, signal, 'b', 'DisplayName', signalName);

y_limits = ylim;

% Plot contact regions
for k = 1:length(l_starts)
    x1 = time_signal(l_starts(k));
    x2 = time_signal(l_ends(k));

    h = patch( ...
        [x1 x2 x2 x1], ...
        [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], ...
        [1.0 0.6 0.6], ...
        'FaceAlpha', 0.3, ...
        'EdgeColor', 'none');

    h.Annotation.LegendInformation.IconDisplayStyle = 'off';
end

% Re-plot signal so it stays on top
plot(time_signal, signal, 'b', 'DisplayName', signalName);

xlabel('Time [s]');
ylabel(signalName, 'Interpreter', 'none');

title([char(signalName) ' inside Left Foot Contact Regions'], ...
    'Interpreter', 'none');

legend;