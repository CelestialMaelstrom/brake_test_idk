clear; close all; clc;

%% 1. Load Data
S1 = load('S1_#31435_20251109_162322.mat');

B_T = S1.Brake_Temperature_Front_Right_Sensor;
t_all = B_T.Time(:);
temp_all = B_T.Value(:);
vel_all = S1.Vehicle_Speed.Value(:) / 3.6;

%% 2. Find "Stationary & Cooling" Zone
% Look for the longest period where:
% 1. Speed is zero (< 0.5 m/s)
% 2. Temperature is dropping
% 3. Temp is significantly above ambient (> 40C)

mask_stationary = (vel_all < 0.5) & (temp_all > 40);

% Find the longest contiguous block of stationary data
diff_mask = diff([0; mask_stationary; 0]);
start_idxs = find(diff_mask == 1);
end_idxs = find(diff_mask == -1) - 1;
[durations, idx] = max(end_idxs - start_idxs);

if isempty(durations) || durations < 50
    error('Could not find a long enough stationary cool-down period.');
end

range = start_idxs(idx):end_idxs(idx);
t_seg = t_all(range);
T_seg = temp_all(range);

% Shift time to start at 0 for fitting
t_fit = t_seg - t_seg(1);

%% 3. Fit Exponential Decay (Robust Physics)
% Model: T(t) = Tamb + (Tstart - Tamb) * e^(-t / tau)
% tau = (m * c) / (h_total * A)

params = buildParams();
T_amb = params.T_amb;

% Linearize for fitting: ln(T - Tamb) = ln(dT_0) - (1/tau)*t
Y = log(T_seg - T_amb);
p = polyfit(t_fit, Y, 1); 

slope = p(1); % This is -1/tau
tau_measured = -1 / slope;

% Check fit quality
T_model = T_amb + (T_seg(1) - T_amb) * exp(-t_fit / tau_measured);
r_squared = 1 - sum((T_seg - T_model).^2) / sum((T_seg - mean(T_seg)).^2);

%% 4. Back-Calculate h_static
% tau = m*c / (h*A)  ->  h = m*c / (tau*A)
% Note: This 'h' includes Linearized Radiation, but at low temps 
% Radiation is small, so this is a safe "Effective h".

m = params.m_r; 
c = params.c_r; 
A = params.A_rotor;

h_total_static = (m * c) / (tau_measured * A);

%% 5. Construct New Cooling Table
% We anchor the curve at 0 km/h using our measured value.
% We add a standard FSAE Nusselt slope for speed (approx +0.5 h per km/h)
% or use a conservative slope of +0.4 to +0.6 for shrouded wheels.

slope_h_v = 0.4; % W/m2K per km/h (Conservative Estimate)

v_breakpoints = [0 20 40 60 80 100];
h_new = h_total_static + (slope_h_v * v_breakpoints);

%% 6. Plot & Output
figure('Color','w');
subplot(2,1,1);
plot(t_fit, T_seg, 'b.', 'DisplayName', 'Measured Data'); hold on;
plot(t_fit, T_model, 'r-', 'LineWidth', 2, 'DisplayName', 'Exponential Fit');
title(sprintf('Stationary Decay Fit (R^2 = %.3f)', r_squared));
xlabel('Time (s)'); ylabel('Temp (C)');
legend(); grid on;

subplot(2,1,2);
plot(v_breakpoints, h_new, 'g-o', 'LineWidth', 2);
title('New Calibrated Cooling Curve');
xlabel('Speed (km/h)'); ylabel('h (W/m^2K)');
grid on;
ylim([0 max(h_new)*1.2]);

fprintf('\n========================================\n');
fprintf('       STATIONARY CALIBRATION RESULTS     \n');
fprintf('========================================\n');
fprintf('Measured Time Constant (tau): %.1f seconds\n', tau_measured);
fprintf('Calculated h_static (v=0):    %.2f W/m^2K\n', h_total_static);
fprintf('----------------------------------------\n');
fprintf('Paste this into buildParams.m:\n\n');
fprintf('params.h_vel   = %s;\n', mat2str(v_breakpoints));
fprintf('params.h_Wm2K  = %s;\n', mat2str(h_new, 3));
fprintf('========================================\n');