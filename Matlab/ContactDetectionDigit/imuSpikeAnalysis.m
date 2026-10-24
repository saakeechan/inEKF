clear;
close all; clc

%% Load
T = readtable(fullfile('..','digit_cropped_data_full.csv'));

tname = 'TimeStamp';
n = min(height(T), 977);

time = T.(tname)(1:n);

% --- Ground-truth contacts ---
l_gt = logical(T.lfoot_contact(1:n));
r_gt = logical(T.rfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);
[r_starts, r_ends] = get_regions(r_gt);

%% Signal + causal moving average
imu_signal = T.acc_y(1:n);

w = 5;                              % window in samples
window_size = w;
dt = 0.005;
x = movmean(imu_signal, [w-1 0]);    % causal moving average

figure; hold on; grid on;

% --- Plot IMU first ---
plot(time, x, 'k-');
yl = ylim;

% --- GT patches (left = red) ---
for k = 1:length(l_starts)
    patch([time(l_starts(k)) time(l_ends(k)) time(l_ends(k)) time(l_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [1 0.6 0.6], 'FaceAlpha', 0.25, 'EdgeColor','none');
end

% --- GT patches (right = green) ---
for k = 1:length(r_starts)
    patch([time(r_starts(k)) time(r_ends(k)) time(r_ends(k)) time(r_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [0.6 1 0.6], 'FaceAlpha', 0.25, 'EdgeColor','none');
end

% Re-plot IMU on top
% plot(time, x, 'k-');

xlabel('Time');
yyaxis left
ylabel('IMU acc\_x (causal movmean)');
title('IMU acc\_x with GT contacts + IMU slope');

%% =========================
%  Rolling slope of IMU via local linear regression
% =========================
y = x(:);
N = min(numel(y), numel(time));
y = y(1:N);
tS = time(1:N);

W = window_size;     % e.g., 10, 20, 50
slope = zeros(N,1);

for t = 1:N
    sIdx = max(1, t - W + 1);
    yy   = y(sIdx:t);
    kk   = (0:numel(yy)-1)';     % local index
    kk0  = kk - mean(kk);

    slope(t) = (kk0'*(yy - mean(yy))) / max(sum(kk0.^2), eps); % per-sample
end

% Convert to per-second (dt must match your sampling)
s = slope / dt;

%% --- Overlay IMU slope on right axis ---
yyaxis right
% plot(tS, s, 'b-');
ylabel('IMU slope (units/s)');

hold off