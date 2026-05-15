clc;
close all;

%% Load data
T = readtable('/home/saakee/Research/Saaketh_INEKF/Data/G1LoggingData.csv');

%% Parameters
tname = 'timestamp';
dataName = 'gyro_body_y';   % <-- change signal here

n = min(height(T), 500);

time = T.(tname)(1:n);

%% Ground-truth contacts
l_gt = logical(T.lfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);

%% Extract selected signal
signal = T.(dataName)(1:n);

signal = abs(signal);

%% Plot
figure;
hold on;
grid on;

% Plot signal
plot(time, signal, 'b', 'DisplayName', dataName);

% Get axis limits
y_limits = ylim;

%% Plot contact regions in RED (hidden from legend)
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