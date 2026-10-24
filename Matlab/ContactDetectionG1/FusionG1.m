%% Fusion

S = load("torqueDetectionG1.mat");
DV = load("dvDetectionG1.mat");

pL_torque = S.pL_contact(:);

pR_torque = S.pR_contact(:);

pL_dv = DV.pDV_leg1(:);
pL_dv = [0.018; pL_dv(1:end-1)];  % prepend 0.018 and remove last value

pR_dv = DV.pDV_leg2(:);
pR_dv = [0.018; pR_dv(1:end-1)];  % prepend 0.018 and remove last value

%% =========================
% Fuse probabilities (normalized weights)
% =========================
w_tau = 0.7;
w_dv  = 0.3;

wsum = w_tau + w_dv;
w_tau = w_tau/wsum;  

w_dv  = w_dv/wsum;

pR_total = w_tau*pR_torque + w_dv*pR_dv;
pL_total = w_tau*pL_torque + w_dv*pL_dv;

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