clc;close all

%% Load
T = readtable(fullfile('../..','digit_cropped_data_full.csv'));

tname = 'TimeStamp';
n = min(height(T), 976);

time = T.(tname)(1:n);

% --- Ground-truth contacts ---
l_gt = logical(T.lfoot_contact(1:n));
r_gt = logical(T.rfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);
[r_starts, r_ends] = get_regions(r_gt);

dt = 0.005;



%% Signal + causal moving average
imu_signal = T.gyro_z(1:n);

w = 5;                              % window in samples
x = movmean(imu_signal, [w-1 0]);    % causal moving average

%%
% --- Probabilities from signed squared distance ---
x0 = 2;        % "confident" magnitude (in units of x)
p0 = 0.95;     % desired probability at +x0
k  = log(p0/(1-p0)) / (x0^2);

z = k * (x .* abs(x));           % signed square: x|x|
z = max(min(z, 10), -10);        % optional clamp

p_r = 1 ./ (1 + exp(-z));        % P(right)
p_l = 1 - p_r;                   % P(left)


%% Plot: signal + GT patches + predicted patches
figure; hold on; grid on;

% Plot signal FIRST so ylim is valid
plot(time, x, 'k-');   % filtered / classified signal
yl = ylim;

% --- GT patches (left=red, right=green) ---
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

xlabel('Time');
ylabel('acc\_y (causal movmean)');
title('IMU Schmitt Trigger Contact Classification (GT vs Pred)');

%% Threshold probabilities -> logical predictions
th = 0.5;              % choose threshold

r_hat = (p_r >= th);
l_hat = (p_l >= th);   % same as ~r_hat if p_l = 1 - p_r

%% Compare logical vectors (agreement rate)
r_gt = logical(r_gt(:));
l_gt = logical(l_gt(:));

r_hat = logical(r_hat(:));
l_hat = logical(l_hat(:));

agree_r = mean(r_hat == r_gt) * 100;
agree_l = mean(l_hat == l_gt) * 100;

mismatch_r = nnz(r_hat ~= r_gt);
mismatch_l = nnz(l_hat ~= l_gt);

mismatch_pct_r = 100 * mismatch_r / numel(r_gt);
mismatch_pct_l = 100 * mismatch_l / numel(l_gt);

fprintf('\n=== Logical agreement (percent) ===\n');
fprintf('Right agree      = %.2f %%\n', agree_r);
fprintf('Left  agree      = %.2f %%\n', agree_l);




% %% Verification vs ground truth (sample-wise)
% % Confusion matrices per foot
% TP_L = sum(l_hat & l_gt);
% FP_L = sum(l_hat & ~l_gt);
% FN_L = sum(~l_hat & l_gt);
% TN_L = sum(~l_hat & ~l_gt);
% 
% TP_R = sum(r_hat & r_gt);
% FP_R = sum(r_hat & ~r_gt);
% FN_R = sum(~r_hat & r_gt);
% TN_R = sum(~r_hat & ~r_gt);
% 
% % Metrics
% acc_L = (TP_L + TN_L) / numel(l_gt);
% prec_L = TP_L / max(TP_L + FP_L, 1);
% rec_L  = TP_L / max(TP_L + FN_L, 1);
% f1_L   = 2 * prec_L * rec_L / max(prec_L + rec_L, 1e-12);
% 
% acc_R = (TP_R + TN_R) / numel(r_gt);
% prec_R = TP_R / max(TP_R + FP_R, 1);
% rec_R  = TP_R / max(TP_R + FN_R, 1);
% f1_R   = 2 * prec_R * rec_R / max(prec_R + rec_R, 1e-12);
% 
% fprintf('\n=== Verification (sample-wise) ===\n');
% fprintf('Left  : Acc=%.3f  Prec=%.3f  Rec=%.3f  F1=%.3f   (TP=%d FP=%d FN=%d TN=%d)\n', ...
%     acc_L, prec_L, rec_L, f1_L, TP_L, FP_L, FN_L, TN_L);
% fprintf('Right : Acc=%.3f  Prec=%.3f  Rec=%.3f  F1=%.3f   (TP=%d FP=%d FN=%d TN=%d)\n', ...
%     acc_R, prec_R, rec_R, f1_R, TP_R, FP_R, FN_R, TN_R);

save('IMUTest1.mat');