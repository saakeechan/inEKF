clear;clc;close all;

T_dv = readtable("/home/saakee/Research/Saaketh_INEKF/build/G1_dv_values.csv");
dv_leg1 = T_dv{:,1};
dv_leg2 = T_dv{:,2};

T = readtable(fullfile("/home/saakee/Research/Saaketh_INEKF/G1LoggingData.csv"));

n = min(height(T), 100);

% --- Ground-truth contacts ---
l_gt = logical(T.lfoot_contact(1:n));
r_gt = logical(T.rfoot_contact(1:n));

l_gt_indices = find(l_gt);
dv_leg1_inContact = dv_leg1(l_gt_indices);

r_gt_indices = find(r_gt);
dv_leg2_inContact = dv_leg2(r_gt_indices);

sorted_leg_1 = sort(dv_leg1_inContact,'descend');

median(sorted_leg_1)
median(dv_leg2_inContact)

%% =========================
% Asymmetric DV -> contact probability (piecewise)
% =========================

dv0 = 0.3;        % midpoint: p = 0.5
zclip = 4;

% --- Anchors you actually mean ---
dv_contact    = 0.10;   p_contact    = 0.95;  % confident contact below dv0
dv_nocontact  = 0.7;   p_nocontact  = 0.05;  % confident NO contact above dv0

% Distances from dv0
dL = dv0 - dv_contact;      % >0
dU = dv_nocontact - dv0;    % >0

% Map anchors through logit: log(p/(1-p))
aL = log(p_contact/(1-p_contact));           % positive
aU = log(p_nocontact/(1-p_nocontact));       % negative

% Gains so that:
% for dv = dv0 - dL => z = +aL
% for dv = dv0 + dU => z =  aU  (negative)
kL = aL / (dL^2);           % >0
kU = (-aU) / (dU^2);        % >0  (note minus because aU is negative)

dv2prob_asym = @(dv) local_dv2prob_asym(dv, dv0, kL, kU, zclip);

% Probabilities
pDV_leg1 = dv2prob_asym(dv_leg1);
pDV_leg2 = dv2prob_asym(dv_leg2);

%% ---- local function (put at end of script) ----
function p = local_dv2prob_asym(dv, dv0, kL, kU, zclip)
    dv = dv(:);
    d  = dv - dv0;

    z = zeros(size(d));

    % Below dv0: want high contact prob => positive z
    idxL = (d <= 0);
    z(idxL) = -kL * (d(idxL) .* abs(d(idxL)));   % d*|d| is negative => z positive

    % Above dv0: want low contact prob => negative z
    idxU = ~idxL;
    z(idxU) = -kU * (d(idxU) .* abs(d(idxU)));   % d*|d| positive => z negative

    z = max(min(z, zclip), -zclip);
    p = 1 ./ (1 + exp(-z));
end

%% =========================
% Threshold probability
% =========================

prob_thresh = 0.5;   % you can tune this

pred_leg1 = pDV_leg1 >= prob_thresh;   % logical 1/0

N = min(length(pred_leg1), length(l_gt));

pred_leg1 = pred_leg1(1:N);
l_gt      = l_gt(1:N);

TP = sum( pred_leg1 == 1 & l_gt == 1 );
TN = sum( pred_leg1 == 0 & l_gt == 0 );
FP = sum( pred_leg1 == 1 & l_gt == 0 );
FN = sum( pred_leg1 == 0 & l_gt == 1 );

accuracy  = (TP + TN) / N;

precision = TP / max(TP + FP, eps);
recall    = TP / max(TP + FN, eps);
F1        = 2 * (precision * recall) / max(precision + recall, eps);

fprintf('\n--- Leg 1 Contact Detection ---\n');
fprintf('TP: %d\n', TP);
fprintf('TN: %d\n', TN);
fprintf('FP: %d\n', FP);
fprintf('FN: %d\n', FN);

fprintf('Accuracy : %.4f\n', accuracy);
fprintf('Precision: %.4f\n', precision);
fprintf('Recall   : %.4f\n', recall);
fprintf('F1 Score : %.4f\n', F1);

%% =========================
% Threshold probability (Leg 2)
% =========================

prob_thresh = 0.5;   % same threshold for fairness

pred_leg2 = pDV_leg2 >= prob_thresh;   % logical 1/0

N2 = min(length(pred_leg2), length(r_gt));

pred_leg2 = pred_leg2(1:N2);
r_gt2     = r_gt(1:N2);

TP2 = sum( pred_leg2 == 1 & r_gt2 == 1 );
TN2 = sum( pred_leg2 == 0 & r_gt2 == 0 );
FP2 = sum( pred_leg2 == 1 & r_gt2 == 0 );
FN2 = sum( pred_leg2 == 0 & r_gt2 == 1 );

accuracy2  = (TP2 + TN2) / N2;

precision2 = TP2 / max(TP2 + FP2, eps);
recall2    = TP2 / max(TP2 + FN2, eps);
F12        = 2 * (precision2 * recall2) / max(precision2 + recall2, eps);

fprintf('\n--- Leg 2 Contact Detection ---\n');
fprintf('TP: %d\n', TP2);
fprintf('TN: %d\n', TN2);
fprintf('FP: %d\n', FP2);
fprintf('FN: %d\n', FN2);

fprintf('Accuracy : %.4f\n', accuracy2);
fprintf('Precision: %.4f\n', precision2);
fprintf('Recall   : %.4f\n', recall2);
fprintf('F1 Score : %.4f\n', F12);

% save("dVAnalysis.mat")

% --- NOT in contact masks ---
l_nocontact = ~l_gt;
r_nocontact = ~r_gt;

% --- Extract DV when NOT in contact ---
dv_leg1_notContact = dv_leg1(l_nocontact);
dv_leg2_notContact = dv_leg2(r_nocontact);

% --- Sort (optional) ---
sorted_leg1_not = sort(dv_leg1_notContact, 'descend');
sorted_leg2_not = sort(dv_leg2_notContact, 'descend');

% --- Compare medians ---
median_in_L  = median(dv_leg1_inContact);
median_out_L = median(dv_leg1_notContact);

median_in_R  = median(dv_leg2_inContact);
median_out_R = median(dv_leg2_notContact);

fprintf('Leg1 median DV: in-contact=%.4f, not-contact=%.4f\n', median_in_L, median_out_L);
fprintf('Leg2 median DV: in-contact=%.4f, not-contact=%.4f\n', median_in_R, median_out_R);

%% =========================
% Plot DV with GT contact overlays (Left + Right)
% =========================

% Make sure lengths match n
dv_leg1_plot = dv_leg1(1:n);
dv_leg2_plot = dv_leg2(1:n);

l_gt_plot = l_gt(1:n);
r_gt_plot = r_gt(1:n);

t = (1:n)';  % sample index; replace with time vector if you have it

figure('Name','DV vs Ground-Truth Contact','Color','w');

%% =========================
% LEFT LEG
% =========================

figure; hold on; grid on;
title('Left Leg: DV with Ground-Truth Contact');
xlabel('Sample');
ylabel('DV');

mn1 = min(dv_leg1_plot);
mx1 = max(dv_leg1_plot);
span1 = mx1 - mn1;
if span1 < eps, span1 = 1; end

yl = [mn1 mx1] + 0.05*[-1 1]*span1;
ylim(yl);

plot_contact_patches(t, l_gt_plot, yl);

plot(t, dv_leg1_plot, 'k');

gt_line = yl(1) + 0.05*span1 + 0.10*span1*double(l_gt_plot);
plot(t, gt_line, 'r');

legend({'GT contact region','DV','GT (scaled)'});


%% =========================
% RIGHT LEG
% =========================

figure; hold on; grid on;
title('Right Leg: DV with Ground-Truth Contact');
xlabel('Sample');
ylabel('DV');

mn2 = min(dv_leg2_plot);
mx2 = max(dv_leg2_plot);
span2 = mx2 - mn2;
if span2 < eps, span2 = 1; end

yl2 = [mn2 mx2] + 0.05*[-1 1]*span2;
ylim(yl2);

plot_contact_patches(t, r_gt_plot, yl2);

plot(t, dv_leg2_plot, 'k');

gt_line2 = yl2(1) + 0.05*span2 + 0.10*span2*double(r_gt_plot);
plot(t, gt_line2, 'r');

legend({'GT contact region','DV','GT (scaled)'});


%% ---- Helper: contact patches (put at end of script) ----
function plot_contact_patches(t, gt, yl)
    gt = gt(:) ~= 0;
    t  = t(:);

    % Find rising/falling edges of contact
    d = diff([false; gt; false]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;

    for k = 1:numel(starts)
        xs = [t(starts(k)) t(ends(k)) t(ends(k)) t(starts(k))];
        ys = [yl(1) yl(1) yl(2) yl(2)];
        patch(xs, ys, 0.85*[1 1 1], 'EdgeColor','none', 'FaceAlpha',0.35);
    end
end

save("dvDetectionG1.mat")