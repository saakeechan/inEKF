clc;close all;

%% Load
T = readtable(fullfile('/home/saakee/Research/Saaketh_INEKF/G1LoggingData.csv'));

tname = 'timestamp';
n = min(height(T), 300);

time = T.(tname)(1:n);

% --- Ground-truth contacts ---
l_gt = logical(T.lfoot_contact(1:n));
r_gt = logical(T.rfoot_contact(1:n));

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);
[r_starts, r_ends] = get_regions(r_gt);

dt = 0.02;

% --- Build torque matrices (measured) ---

cols_left_tau = { ...
    'joint_torque_left_hip_roll_joint', ...
    'joint_torque_left_hip_yaw_joint', ...
    'joint_torque_left_hip_pitch_joint', ...      
    'joint_torque_left_knee_joint', ...
    'joint_torque_left_ankle_pitch_joint', ...
    'joint_torque_left_ankle_roll_joint' };

cols_right_tau = { ...
    'joint_torque_right_hip_roll_joint', ...
    'joint_torque_right_hip_yaw_joint', ...
    'joint_torque_right_hip_pitch_joint', ...    
    'joint_torque_right_knee_joint', ...
    'joint_torque_right_ankle_pitch_joint', ...
    'joint_torque_right_ankle_roll_joint' };

L = zeros(numel(cols_left_tau),n);
R = zeros(numel(cols_right_tau),n);
moving_avg_Left = [];
moving_avg_Right = [];

window_size_time = dt * 1;
window_size = int32(window_size_time/dt);


for t = 1:n
    for i = 1:numel(cols_left_tau)
        LeftTorquesAllLegs(i,t) = double(T.(cols_left_tau{i})(t));
        RightTorquesAllLegs(i,t) = double(T.(cols_right_tau{i})(t));
    end
    LeftTorquesCombined(t) = sum(LeftTorquesAllLegs(:,t).^2);
    RightTorquesCombined(t) = sum(RightTorquesAllLegs(:,t).^2);

start_idx = max(1, t - window_size + 1);

moving_avg_Left(t)  = mean(LeftTorquesCombined(start_idx:t));
moving_avg_Right(t) = mean(RightTorquesCombined(start_idx:t));

end

% 
figure
plot(time, moving_avg_Left, 'b')
hold on
% plot(time, moving_avg_Right, 'r')
grid on

yl = ylim;   % lock y-limits AFTER plotting signals

% --- Left contact patches (red) ---
for k = 1:length(l_starts)
    patch([time(l_starts(k)) time(l_ends(k)) ...
           time(l_ends(k))   time(l_starts(k))], ...
          [yl(1) yl(1) yl(2) yl(2)], ...
          [1 0.6 0.6], ...
          'FaceAlpha', 0.3, ...
          'EdgeColor','none');
end

% % --- Right contact patches (green) ---
% for k = 1:length(r_starts)
%     patch([time(r_starts(k)) time(r_ends(k)) ...
%            time(r_ends(k))   time(r_starts(k))], ...
%           [yl(1) yl(1) yl(2) yl(2)], ...
%           [0.6 1 0.6], ...
%           'FaceAlpha', 0.3, ...
%           'EdgeColor','none');
% end
%% Slope Calculator


yR = moving_avg_Right(:);
yL = moving_avg_Left(:);

NR = numel(yR);
NL = numel(yL);
N  = min(NR, NL);     % evaluate both on common length
yR = yR(1:N);
yL = yL(1:N);

W = window_size;      % e.g. 10, 20, 50

slopeR = zeros(N,1);
slopeL = zeros(N,1);

%% =========================
%  Rolling-slope via local linear regression (Right + Left)
% =========================
for t = 1:N
    sIdx = max(1, t - W + 1);

    % ---- Right ----
    yyR = yR(sIdx:t);
    kk  = (0:numel(yyR)-1)';          % local index
    kk0 = kk - mean(kk);
    slopeR(t) = (kk0'*(yyR - mean(yyR))) / max(sum(kk0.^2), eps);

    % ---- Left ----
    yyL = yL(sIdx:t);
    kk  = (0:numel(yyL)-1)';          % local index
    kk0 = kk - mean(kk);
    slopeL(t) = (kk0'*(yyL - mean(yyL))) / max(sum(kk0.^2), eps);
end

%% =========================
%  Signals (convert slope to per-second)
% =========================
sR = slopeR / dt;
sL = slopeL / dt;

%%
%% =========================
%  ASYMMETRIC TORQUE -> CONTACT PROBABILITY (+ slope bias)
%  p(T0)=0.5, p(T_high)=p_high, p(T_low)=p_low
% =========================

% ---- Midpoint + anchors ----
T0     = 850;       % midpoint: p(T0)=0.5

T_high = 2000;    % confident contact point (high torque)
p_high = 0.95;

T_low  = 100;       % confident NO-contact point (low torque)
p_low  = 0.05;

% ---- Compute asymmetric quadratic gains ----
d_high = T_high - T0;          % >0
d_low  = T0 - T_low;           % >0

a_high = log(p_high/(1-p_high));   % >0
a_low  = log(p_low /(1-p_low ));   % <0

k_high = a_high / (d_high^2);      % curvat0re for T > T0
k_low  = (-a_low) / (d_low^2);     % curvature for T < T0  (positive)

% ---- Slope bias tuning (same as before) ----
Dgate   = 500;     % torque units: "near threshold" width
S0      = 1e5;      % slope units: saturation scale (torque/sec)
k_slope = 0;      % max logit bump near threshold

% ---- Utility: clamp logits ----
zclip = 10;

%% =========================
% LEFT
%% =========================
dL = moving_avg_Left(:) - T0;   % signed distance

% Asymmetric torque logit
zTL = zeros(size(dL));
idxH = (dL >= 0);
idxL = ~idxH;

zTL(idxH) =  k_high * (dL(idxH).^2);   % above threshold -> positive logit
zTL(idxL) = -k_low  * (dL(idxL).^2);   % below threshold -> negative logit

% Hard gate: slope only matters when |T - T0| <= Dgate
gL  = (abs(dL) <= Dgate);           % logical 0/1
uL  = tanh(sL(:)/S0);               % [-1, 1]
zSL = k_slope * double(gL) .* uL;   % slope logit (only inside band)

zL = zTL + zSL;
zL = max(min(zL, zclip), -zclip);
pL_contact = 1 ./ (1 + exp(-zL));

%% =========================
% RIGHT
%% =========================
dR = moving_avg_Right(:) - T0;

% Asymmetric torque logit
zTR = zeros(size(dR));
idxH = (dR >= 0);
idxL = ~idxH;

zTR(idxH) =  k_high * (dR(idxH).^2);
zTR(idxL) = -k_low  * (dR(idxL).^2);

gR  = (abs(dR) <= Dgate);
uR  = tanh(sR(:)/S0);
zSR = k_slope * double(gR) .* uR;

zR = zTR + zSR;
zR = max(min(zR, zclip), -zclip);
pR_contact = 1 ./ (1 + exp(-zR));

%% =========================
% CONTACT CLASSIFICATION
%% =========================

contact_left  = pL_contact >= 0.5;
contact_right = pR_contact >= 0.5;

%% =========================
% CONFUSION MATRIX ANALYSIS
%% =========================

% ---- LEFT LEG ----
TP_L = sum((contact_left  == 1) & (l_gt == 1));
TN_L = sum((contact_left  == 0) & (l_gt == 0));
FP_L = sum((contact_left  == 1) & (l_gt == 0));
FN_L = sum((contact_left  == 0) & (l_gt == 1));

% Metrics
acc_L  = (TP_L + TN_L) / (TP_L + TN_L + FP_L + FN_L);
prec_L = TP_L / (TP_L + FP_L);
rec_L  = TP_L / (TP_L + FN_L);
F1_L   = 2 * (prec_L * rec_L) / (prec_L + rec_L);

% ---- RIGHT LEG ----
TP_R = sum((contact_right == 1) & (r_gt == 1));
TN_R = sum((contact_right == 0) & (r_gt == 0));
FP_R = sum((contact_right == 1) & (r_gt == 0));
FN_R = sum((contact_right == 0) & (r_gt == 1));

% Metrics
acc_R  = (TP_R + TN_R) / (TP_R + TN_R + FP_R + FN_R);
prec_R = TP_R / (TP_R + FP_R);
rec_R  = TP_R / (TP_R + FN_R);
F1_R   = 2 * (prec_R * rec_R) / (prec_R + rec_R);

%% Display results
fprintf('\nLEFT LEG\n');
fprintf('TP=%d  TN=%d  FP=%d  FN=%d\n', TP_L, TN_L, FP_L, FN_L);
fprintf('Accuracy=%.3f  Precision=%.3f  Recall=%.3f  F1=%.3f\n', acc_L, prec_L, rec_L, F1_L);

fprintf('\nRIGHT LEG\n');
fprintf('TP=%d  TN=%d  FP=%d  FN=%d\n', TP_R, TN_R, FP_R, FN_R);
fprintf('Accuracy=%.3f  Precision=%.3f  Recall=%.3f  F1=%.3f\n', acc_R, prec_R, rec_R, F1_R);


save torqueDetectionG1.mat