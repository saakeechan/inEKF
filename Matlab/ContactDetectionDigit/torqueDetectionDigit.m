clc;close all

%% Load
T = readtable(fullfile('..','digit_cropped_data_full.csv'));

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

% --- Build torque matrices (measured) ---
cols_left_tau  = { 'left_hip_roll_tau', 'left_hip_yaw_tau', 'left_hip_pitch_tau', ...
                   'left_knee_tau', 'left_toe_A_tau', 'left_toe_B_tau' };

cols_right_tau = { 'right_hip_roll_tau', 'right_hip_yaw_tau', 'right_hip_pitch_tau', ...
                   'right_knee_tau', 'right_toe_A_tau', 'right_toe_B_tau' };

L = zeros(numel(cols_left_tau),n);
R = zeros(numel(cols_right_tau),n);
moving_avg_Left = [];
moving_avg_Right = [];

window_size_time = 0.025;
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
T0     = 5000;       % midpoint: p(T0)=0.5

T_high = 2.25e4;    % confident contact point (high torque)
p_high = 0.95;

T_low  = 200;       % confident NO-contact point (low torque)
p_low  = 0.05;

% ---- Compute asymmetric quadratic gains ----
d_high = T_high - T0;          % >0
d_low  = T0 - T_low;           % >0

a_high = log(p_high/(1-p_high));   % >0
a_low  = log(p_low /(1-p_low ));   % <0

k_high = a_high / (d_high^2);      % curvat0re for T > T0
k_low  = (-a_low) / (d_low^2);     % curvature for T < T0  (positive)

% ---- Slope bias tuning (same as before) ----
Dgate   = 4000;     % torque units: "near threshold" width
S0      = 1e4;      % slope units: saturation scale (torque/sec)
k_slope = 2;      % max logit bump near threshold

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

%% Fusion

S = load("IMUTest1.mat");
DV = load("dVAnalysis.mat");

pR_imu = S.p_r(:);
pL_imu = S.p_l(:);

pR_dv = DV.pDV_leg2(:);
pL_dv = DV.pDV_leg1(:);

%% =========================
% Fuse probabilities (normalized weights)
% =========================
w_tau = 0.65;
w_imu = 0;
w_dv  = 0.35;

wsum = w_tau + w_imu + w_dv;
w_tau = w_tau/wsum;  
w_imu = w_imu/wsum;  
w_dv  = w_dv/wsum;

pR_total = w_tau*pR_contact + w_imu*pR_imu + w_dv*pR_dv;
pL_total = w_tau*pL_contact + w_imu*pL_imu + w_dv*pL_dv;

% Hard decisions
th = 0.5;
predR = (pR_total >= th);
predL = (pL_total >= th);

%% =========================
% Ground truth (Right + Left)
% =========================

gtR_all = r_gt(:);
gtL_all = l_gt(:);

% Ensure consistent length across everything
Ngt = min([ ...
    numel(predR), numel(predL), ...
    numel(gtR_all), numel(gtL_all) ...
]);

% Truncate consistently
predR = predR(1:Ngt);
predL = predL(1:Ngt);
gtR   = gtR_all(1:Ngt);
gtL   = gtL_all(1:Ngt);

%% =========================
% Metrics + report
% =========================
metrics = @(pred,gt) struct( ...
    'TP', sum(pred & gt), ...
    'TN', sum(~pred & ~gt), ...
    'FP', sum(pred & ~gt), ...
    'FN', sum(~pred & gt) );

% Right
mR = metrics(predR, gtR);
accR  = (mR.TP + mR.TN) / Ngt;
precR = mR.TP / max(mR.TP + mR.FP, 1);
recR  = mR.TP / max(mR.TP + mR.FN, 1);
f1R   = (2*mR.TP) / max(2*mR.TP + mR.FP + mR.FN, 1);

% Left
mL = metrics(predL, gtL);
accL  = (mL.TP + mL.TN) / Ngt;
precL = mL.TP / max(mL.TP + mL.FP, 1);
recL  = mL.TP / max(mL.TP + mL.FN, 1);
f1L   = (2*mL.TP) / max(2*mL.TP + mL.FP + mL.FN, 1);

fprintf('--- FUSED contact estimate vs GT (RIGHT) ---\n');
fprintf('Accuracy:  %.2f%%\n', 100*accR);
fprintf('Precision: %.2f%%\n', 100*precR);
fprintf('Recall:    %.2f%%\n', 100*recR);
fprintf('F1:        %.2f%%\n', 100*f1R);
fprintf('Confusion: TP=%d, FP=%d, TN=%d, FN=%d (N=%d)\n\n', ...
    mR.TP, mR.FP, mR.TN, mR.FN, Ngt);

fprintf('--- FUSED contact estimate vs GT (LEFT) ---\n');
fprintf('Accuracy:  %.2f%%\n', 100*accL);
fprintf('Precision: %.2f%%\n', 100*precL);
fprintf('Recall:    %.2f%%\n', 100*recL);
fprintf('F1:        %.2f%%\n', 100*f1L);
fprintf('Confusion: TP=%d, FP=%d, TN=%d, FN=%d (N=%d)\n', ...
    mL.TP, mL.FP, mL.TN, mL.FN, Ngt);