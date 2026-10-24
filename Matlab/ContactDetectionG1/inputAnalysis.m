clc;
close all;

%% Load data
T = readtable('/home/saakee/Research/Saaketh_INEKF/Data/robotstate_0_env2.csv');

%% Parameters
tname = 'timestamp';

n = height(T);

time = T.(tname)(1:n);
time = time(:);

dt = median(diff(time));
fs = 1 / dt;

fprintf('Estimated fs = %.3f Hz\n', fs);

%% Ground-truth contacts

varNames = string(T.Properties.VariableNames);

disp("Available contact-like columns:");
disp(varNames(contains(lower(varNames), "contact"))');

% Pick contact column manually here if needed.
contact_col = "lfoot_contact";

if ~any(varNames == contact_col)
    error("Column '%s' not found. Check the printed contact-like columns above.", contact_col);
end

raw_contact = T.(contact_col)(1:n);

% Convert table column to numeric if needed.
if iscell(raw_contact) || isstring(raw_contact) || ischar(raw_contact)
    raw_contact = str2double(string(raw_contact));
end

raw_contact = double(raw_contact(:));

fprintf('\nContact column: %s\n', contact_col);
fprintf('Unique contact values:\n');
disp(unique(raw_contact)');

fprintf('Number of samples with contact > 0.5: %d out of %d\n', ...
    sum(raw_contact > 0.5), length(raw_contact));

% Robust binary conversion.
l_gt = raw_contact > 0.5;

% Optional cleanup: remove isolated one-sample contact glitches.
l_gt = movmedian(double(l_gt), 3) > 0.5;

get_regions = @(contact) deal( ...
    find(diff([0; contact(:); 0]) == 1), ...
    find(diff([0; contact(:); 0]) == -1) - 1);

[l_starts, l_ends] = get_regions(l_gt);

region_lengths_debug = l_ends - l_starts + 1;

fprintf('\nDetected contact regions before filtering:\n');
disp(table((1:length(l_starts))', l_starts(:), l_ends(:), region_lengths_debug(:), ...
    'VariableNames', {'Region', 'StartIndex', 'EndIndex', 'Length'}));

%% ================================================================
%% Signal: left foot velocity energy
%% ================================================================

% Use 3D FK foot velocity energy instead of only x velocity.
% This is better because slip can appear in x, y, or z.
vx = T.fk_left_foot_vel_x(1:n);
vy = T.fk_left_foot_vel_y(1:n);
vz = T.fk_left_foot_vel_z(1:n);

signal = vx.^2 + vy.^2 + vz.^2;
signal = signal(:);

dataName = 'left foot FK velocity energy';

%% Plot original energy signal with contacts
figure;
hold on;
grid on;

plot(time, signal, 'b', 'DisplayName', dataName);

y_limits = ylim;

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

plot(time, signal, 'b', 'DisplayName', dataName);

xlabel('Time [s]');
ylabel(dataName, 'Interpreter', 'none');
title([dataName ' with Left Foot Contact Regions'], 'Interpreter', 'none');
legend;

%% ================================================================
%% Energy-Waveform Slip Detection
%% ================================================================

min_region_len = 3;
N_phase = 128;

% Energy smoothing window in normalized stance samples.
% Larger = smoother energy envelope.
energy_smooth_window = 2;

% Robust threshold aggressiveness.
% Lower = more sensitive.
k_mad = 1.0;

region_lengths = l_ends - l_starts + 1;
valid_regions = region_lengths >= min_region_len;

l_starts_valid = l_starts(valid_regions);
l_ends_valid = l_ends(valid_regions);

num_regions = length(l_starts_valid);

if num_regions < 2
    error('Need at least 2 valid contact regions for energy-waveform comparison.');
elseif num_regions == 2
    warning('Only 2 valid contact regions found. Template will be weak.');
end

%% Build phase-normalized energy waveforms

E_raw = zeros(num_regions, N_phase);
E_norm = zeros(num_regions, N_phase);

for i = 1:num_regions
    idx_start = l_starts_valid(i);
    idx_end = l_ends_valid(i);

    e_contact = signal(idx_start:idx_end);

    % Normalize every stance to the same phase grid.
    e_phase = resample_contact_to_phase(e_contact, N_phase);

    % Smooth the energy envelope.
    e_phase = movmean(e_phase, energy_smooth_window);

    E_raw(i, :) = e_phase;

    % Robustly normalize energy shape per stance.
    % This makes it less sensitive to gait/speed amplitude changes.
    E_norm(i, :) = robust_normalize_positive_vector(e_phase);
end

%% Leave-one-out energy waveform scoring

scores = zeros(num_regions, 1);

energy_dist_scores = zeros(num_regions, 1);
energy_peak_scores = zeros(num_regions, 1);
energy_area_scores = zeros(num_regions, 1);
energy_concentration_scores = zeros(num_regions, 1);

templates = zeros(num_regions, N_phase);

for i = 1:num_regions

    train_idx = true(num_regions, 1);
    train_idx(i) = false;

    % Leave-one-out normal energy template.
    template = median(E_norm(train_idx, :), 1);
    template = movmedian(template, 7);

    templates(i, :) = template;

    e = E_norm(i, :);

    residual = e - template;

    % 1. RMS energy-envelope mismatch.
    energy_dist = sqrt(mean(residual.^2));

    % 2. Max local energy mismatch.
    energy_peak = max(abs(residual));

    % 3. Total normalized energy mismatch.
    area_e = trapz(e);
    area_template = trapz(template);
    energy_area = abs(area_e - area_template);

    % 4. Energy concentration mismatch.
    % Slip often creates a localized energy burst.
    e_abs = abs(e);
    template_abs = abs(template);

    concentration_e = max(e_abs) / (mean(e_abs) + 1e-8);
    concentration_template = max(template_abs) / (mean(template_abs) + 1e-8);

    energy_concentration = abs(concentration_e - concentration_template);

    energy_dist_scores(i) = energy_dist;
    energy_peak_scores(i) = energy_peak;
    energy_area_scores(i) = energy_area;
    energy_concentration_scores(i) = energy_concentration;

    % Combined slip score.
    scores(i) = ...
        1.5 * energy_dist + ...
        1.0 * energy_peak + ...
        0.5 * energy_area + ...
        0.8 * energy_concentration;
end

%% Robust threshold

threshold = robust_threshold(scores, k_mad);
is_slip = scores > threshold;

fprintf('\nEnergy-waveform slip threshold: %.4f\n', threshold);

results_table = table( ...
    (1:num_regions)', ...
    l_starts_valid(:), ...
    l_ends_valid(:), ...
    time(l_starts_valid(:)), ...
    time(l_ends_valid(:)), ...
    scores, ...
    energy_dist_scores, ...
    energy_peak_scores, ...
    energy_area_scores, ...
    energy_concentration_scores, ...
    is_slip, ...
    'VariableNames', { ...
        'Region', ...
        'StartIndex', ...
        'EndIndex', ...
        'StartTime', ...
        'EndTime', ...
        'SlipScore', ...
        'EnergyDistance', ...
        'EnergyPeakMismatch', ...
        'EnergyAreaMismatch', ...
        'EnergyConcentrationMismatch', ...
        'IsSlip' ...
    } ...
);

disp(results_table);

%% Overlay detected slip regions

figure(1);
hold on;

y_limits = ylim;

for i = 1:num_regions
    if is_slip(i)
        x1 = time(l_starts_valid(i));
        x2 = time(l_ends_valid(i));

        h_slip = patch( ...
            [x1 x2 x2 x1], ...
            [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], ...
            [1.0 0.0 0.0], ...
            'FaceAlpha', 0.45, ...
            'EdgeColor', 'none', ...
            'DisplayName', 'Detected slip');

        h_slip.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

plot(time, signal, 'b', 'DisplayName', dataName);
ylim(y_limits);

%% Plot slip scores

figure;
stem(1:num_regions, scores, 'filled');
grid on;
hold on;

yline(threshold, '--', 'Threshold');

slip_region_ids = find(is_slip);
stem(slip_region_ids, scores(slip_region_ids), 'filled');

xlabel('Contact Region');
ylabel('Slip Score');
title('Energy-Waveform Slip Score');

legend('Slip score', 'Threshold', 'Detected slip');

%% Plot score components

figure;

subplot(4,1,1);
stem(1:num_regions, energy_dist_scores, 'filled');
grid on;
ylabel('RMS mismatch');
title('Energy Score Components');

subplot(4,1,2);
stem(1:num_regions, energy_peak_scores, 'filled');
grid on;
ylabel('Peak mismatch');

subplot(4,1,3);
stem(1:num_regions, energy_area_scores, 'filled');
grid on;
ylabel('Area mismatch');

subplot(4,1,4);
stem(1:num_regions, energy_concentration_scores, 'filled');
grid on;
xlabel('Contact Region');
ylabel('Concentration');

%% Plot normalized energy waveforms

figure;
hold on;
grid on;

phase = linspace(0, 1, N_phase);

for i = 1:num_regions
    if is_slip(i)
        plot(phase, E_norm(i, :), 'r');
    else
        plot(phase, E_norm(i, :), 'Color', [0.6 0.6 0.6]);
    end
end

global_template = median(E_norm(~is_slip, :), 1);

if all(is_slip)
    global_template = median(E_norm, 1);
end

global_template = movmedian(global_template, 7);

plot(phase, global_template, 'k');

xlabel('Normalized Stance Phase');
ylabel('Normalized energy');
title('Phase-Normalized Energy Waveforms vs Robust Template');

legend('Detected slip region(s)', 'Normal-ish regions', 'Energy template');

%% Plot raw energy waveforms

figure;
hold on;
grid on;

for i = 1:num_regions
    if is_slip(i)
        plot(phase, E_raw(i, :), 'r');
    else
        plot(phase, E_raw(i, :), 'Color', [0.6 0.6 0.6]);
    end
end

raw_energy_template = median(E_raw(~is_slip, :), 1);

if all(is_slip)
    raw_energy_template = median(E_raw, 1);
end

raw_energy_template = movmedian(raw_energy_template, 7);

plot(phase, raw_energy_template, 'k');

xlabel('Normalized Stance Phase');
ylabel('Raw energy');
title('Raw Energy Waveforms');

legend('Detected slip region(s)', 'Normal-ish regions', 'Raw energy template');

%% ================================================================
%% Local Functions
%% ================================================================

function x_phase = resample_contact_to_phase(x_contact, N_phase)

    x_contact = x_contact(:);
    n = length(x_contact);

    if n < 2
        x_phase = zeros(1, N_phase);
        return;
    end

    old_phase = linspace(0, 1, n);
    new_phase = linspace(0, 1, N_phase);

    x_phase = interp1(old_phase, x_contact, new_phase, 'linear', 'extrap');
    x_phase = x_phase(:).';
end

function x_norm = robust_normalize_positive_vector(x)

    x = x(:).';

    med_x = median(x);
    mad_x = median(abs(x - med_x));

    x_norm = (x - med_x) ./ (1.4826 * mad_x + 1e-6);

    % Clip to prevent one violent spike from dominating all metrics.
    x_norm = max(min(x_norm, 6), -6);
end

function threshold = robust_threshold(scores, k_mad)

    scores = scores(:);

    med_s = median(scores);
    mad_s = median(abs(scores - med_s)) + 1e-8;

    threshold = med_s + k_mad * mad_s;
end