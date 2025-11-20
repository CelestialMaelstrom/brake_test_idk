clear; close all; clc;

params = buildParams();

%% --- USER SETTINGS ---
% 1. Hydraulic Gain (K_hyd)
%    - Converts Pressure (Pa) -> Braking Force (N)
%    - Found experimentally: 0.00014
params.K_hyd = 0.00014; 

% 2. Cooling Slope
COOLING_SLOPE = 0.50; 

% 3. Static Offset
H_STATIC_OFFSET = 0.0; 
% ----------------------

%% 1. Load Data File
S1 = load('S1_#31435_20251109_162322.mat'); 

%% H Calcs (Auto-Tune)
h_static_meas = auto_tune_cooling(S1, params);
h_base = h_static_meas + H_STATIC_OFFSET;

% Rebuild Cooling Table
v_breakpoints = linspace(0,200,100); % km/h
h_curve = h_base + (COOLING_SLOPE * v_breakpoints);

params.h_vel  = v_breakpoints;
params.h_Wm2K = h_curve;

fprintf('  [Config] Using Cooling Slope: %.2f\n', COOLING_SLOPE);
fprintf('  [Config] Final h Table: %s\n', mat2str(h_curve, 3));

%% Load Channels
% Temp
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t_temp_raw = B_T.Time(:);
temp_raw   = B_T.Value(:);

% Pressure
B_P_F_chan = S1.Brake_Pressure_Front_Sensor; 
t_b_p_raw = B_P_F_chan.Time(:);
b_p_raw   = B_P_F_chan.Value(:);
b_p_raw_Pa = b_p_raw * 6894.76; % PSI -> Pa

% Accel & Vel
accel_chan = S1.IRIMU_V2_IMU_Acceleration_Y_Axis;
t_accel_raw = accel_chan.Time(:);
accel_raw   = accel_chan.Value(:);

vel_chan   = S1.Vehicle_Speed;
t_vel_raw  = vel_chan.Time(:);
vel_raw    = vel_chan.Value(:);

steering_angle = S1.Steering_Angle;
angle_time = steering_angle.Time(:);
angle_val = steering_angle.Value(:);

% Sampling Rates
dt_temp  = mean(diff(t_temp_raw));
dt_accel = mean(diff(t_accel_raw));
dt_vel   = mean(diff(t_vel_raw));

% Master Clock
if dt_accel <= dt_temp && dt_accel <= dt_vel
    t = t_accel_raw;
elseif dt_vel <= dt_temp
    t = t_vel_raw;
else
    t = t_temp_raw;
end

%% 2. Interpolate
temp = interp1(t_temp_raw, temp_raw, t, 'linear', 'extrap');
vel_ms_raw = vel_raw / 3.6; 
vel = interp1(t_vel_raw, vel_ms_raw, t, 'linear', 'extrap');
accel = interp1(t_accel_raw, accel_raw, t, 'linear', 'extrap');
b_p_pa = interp1(t_b_p_raw, b_p_raw_Pa, t, 'linear', 'extrap');
angle_val = interp1(angle_time, angle_val, t, 'linear', 'extrap');

%% 3. Accel Processing
tail_idx = round(0.85 * length(accel));
accel = accel - mean(accel(tail_idx:end));

%% 4. Moving Average
freq = 9/mean(diff(t));
N = round(1.0 * freq); 
if mod(N,2)==0, N=N+1; end 

b = ones(N,1)/N; a = 1;
y = filter(b,a,temp);
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];

%% 5. Run Simulation Loop (PRESSURE LOGIC)
T_predicted = zeros(length(t),1);
T_predicted(1) = temp(1); 

brake_threshold = 10000; % Pa

for i = 2:length(T_predicted)
    dt_step = t(i) - t(i-1);
    if dt_step <= 0, dt_step = 1e-6; end
    
    % CHANGED: Calls 'delta_Temp_P' instead of 'delta_Temp'
    % Note: We no longer pass 'accel' because we trust the pressure sensor!
    dT = delta_Temp_P(T_predicted(i-1), vel(i), b_p_pa(i), params, dt_step, brake_threshold);
    
    T_predicted(i) = T_predicted(i-1) + dT;
end

%% Fig 1
figure;
scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data', 'MarkerFaceAlpha', 0.2); hold on;
plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average');
plot(t, T_predicted, 'g', 'LineWidth', 2, 'DisplayName', 'Predicted Temp (Pressure Model)');
xlabel('Time (s)'); ylabel('Brake Temperature');
legend(); grid on;

%% Fig 2 - Accelerometer
figure;
plot(t, accel, 'b', 'DisplayName', 'Accel');
xlabel('Time (s)'); ylabel('Acceleration (g)');
grid on;

%% Fig 3 - Brake Pressure
figure;
plot(t, b_p_pa, 'm', 'DisplayName', 'Brake Pressure (Pa)');
xlabel('Time (s)'); ylabel('Pressure (Pa)');
grid on;
yline(brake_threshold);

%% Fig 4 -- Overview
figure()
subplot(5,1,1);
scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data', 'MarkerFaceAlpha', 0.2); hold on;
plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average');
plot(t, T_predicted, 'g', 'LineWidth', 2, 'DisplayName', 'Predicted Temp');
xlabel('Time (s)'); ylabel('Brake Temperature');
legend(); grid on;

subplot(5,1,2);
plot(t, vel, 'c', 'DisplayName', 'Velocity (m/s)');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
grid on;

subplot(5,1,3);
plot(t, b_p_pa, 'm', 'DisplayName', 'Brake Pressure (Pa)');
xlabel('Time (s)'); ylabel('Pressure (Pa)');
grid on;
yline(brake_threshold);

subplot(5,1,4);
plot(t, accel, 'b', 'DisplayName', 'Acceleration (g)');

subplot(5,1,5); % Steering Angle
plot(t, angle_val);
xlabel('Time (s)'); ylabel('Steering Angle');