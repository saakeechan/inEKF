clear;
clc;
close all

T = readtable(fullfile('..','digit_cropped_data_full.csv'));

tname  = 'TimeStamp';
n = min(height(T), 977);

time = T.(tname)(1:n);

% --- Contacts (ground truth for overlay) ---
l_contact = logical(T.lfoot_contact(1:n));
r_contact = logical(T.rfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact; 0]) == 1), ...
    find(diff([0; contact; 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_contact);
[r_starts, r_ends] = get_regions(r_contact);

% --- Signal ---
imu_signal = T.acc_x(1:n);

% ===== Causal moving average (optional smoothing) =====
w = 5;
imu_signal_ma = movmean(imu_signal, [w-1 0]);

% ===== VAR METHOD (causal sliding variance) =====
winVar = 25;  % window length in samples (tune)
imu_var = movvar(imu_signal, [winVar-1 0], 0);  % 0 => normalize by N-1 (variance)

% Pick a threshold. Start with something data-driven:
var_thresh = prctile(imu_var, 35);  % tune percentile (e.g., 20-50)

contact_var = imu_var < var_thresh;

% Convert predicted contact to contiguous regions (for patching)
[p_starts, p_ends] = get_regions(contact_var);

% --- Plot base signal FIRST (so ylim is meaningful) ---
figure;
plot(time, imu_signal_ma, 'k-');   % filtered signal
hold on;
grid on;

yl = ylim;

% --- Ground-truth patches (left/right) ---
for k = 1:length(l_starts)
    patch([time(l_starts(k)) time(l_ends(k)) time(l_ends(k)) time(l_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [1 0.6 0.6], 'FaceAlpha', 0.25, 'EdgeColor','none');
end
for k = 1:length(r_starts)
    patch([time(r_starts(k)) time(r_ends(k)) time(r_ends(k)) time(r_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [0.6 1 0.6], 'FaceAlpha', 0.25, 'EdgeColor','none');
end

% --- Predicted contact (variance-based) overlay as gray patches ---
for k = 1:length(p_starts)
    patch([time(p_starts(k)) time(p_ends(k)) time(p_ends(k)) time(p_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [0.7 0.7 0.7], 'FaceAlpha', 0.20, 'EdgeColor','none');
end

% Re-plot line on top
plot(time, imu_signal_ma, 'k-');

ylabel('imuSignal (moving avg)');
xlabel('Time');
legend({'imu\_signal\_ma','GT left','GT right','Pred contact (var)'}, 'Location','best');