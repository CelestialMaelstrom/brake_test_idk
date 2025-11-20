clear; close all; clc;

params = buildParams();

%% --- USER SETTINGS (TWEAK THESE!) ---
% 1. Airflow Slope: How much cooling increases with speed.
%    - Higher = Coots FASTER at speed.
%    - Lower  = Cools SLOWER at speed.
%    - Open Wheel: ~0.6 to 0.8
%    - Fenders/Shrouded: ~0.3 to 0.5
COOLING_SLOPE = 0.35; 

% 2. Static Tweak: Add/Subtract from the auto-calculated baseline
%    - Use 0.0 to trust the data.
%    - Use +2.0 or -2.0 to nudge the curve up/down manually.
H_STATIC_OFFSET = 0.0; 
% -------------------------------------

%% 1. Load Data File
%S1 = load('S1_#31435_20251109_162322.mat'); % Main
S1 = load('S1_#31435_20251109_143045.mat'); % Alternate

%% H Calcs
% --- AUTO-CALIBRATION CALL ---
% This looks at the end of your file to find h at 0 km/h
h_static_meas = auto_tune_cooling(S1, params);

% Apply User Tweaks
h_base = h_static_meas + H_STATIC_OFFSET;

% Rebuild the Cooling Table in Params
v_breakpoints = linspace(0,200,100); % km/h
h_curve = h_base + (COOLING_SLOPE * v_breakpoints);

% Overwrite the params
params.h_vel  = v_breakpoints;
params.h_Wm2K = h_curve;

fprintf('  [Config] Using Cooling Slope: %.2f\n', COOLING_SLOPE);
fprintf('  [Config] Final h Table: %s\n', mat2str(h_curve, 3));

%% Load channels
% Load Raw Channels
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t_temp_raw = B_T.Time(:);
temp_raw   = B_T.Value(:);

% Brake gating
B_P_F_chan = S1.Brake_Pressure_Front_Sensor; 
t_b_p_raw = B_P_F_chan.Time(:);
b_p_raw   = B_P_F_chan.Value(:);

% CORRECTION: Convert Psi -> Pascals
% 1 Psi = 6894.76 Pa
b_p_raw_Pa = b_p_raw * 6894.76;

% Note: Using Y-Axis (Positive = Throttle, Negative = Brake)
accel_chan = S1.IRIMU_V2_IMU_Acceleration_Y_Axis;
t_accel_raw = accel_chan.Time(:);
accel_raw   = accel_chan.Value(:);

vel_chan   = S1.Vehicle_Speed;
t_vel_raw  = vel_chan.Time(:);
vel_raw    = vel_chan.Value(:);

steering_angle = S1.Steering_Angle;
angle_time = steering_angle.Time(:);
angle_val = steering_angle.Value(:);

% Calculate Sampling Times (dt)
dt_temp  = mean(diff(t_temp_raw));
dt_accel = mean(diff(t_accel_raw));
dt_vel   = mean(diff(t_vel_raw));

fprintf('Sampling Intervals:\n  Temp: %.4fs\n  Vel:  %.4fs\n  Acc:  %.4fs\n', dt_temp, dt_vel, dt_accel);

% SELECT MASTER CLOCK (The Fastest Channel)
if dt_accel <= dt_temp && dt_accel <= dt_vel
    t = t_accel_raw;
    fprintf('>> Master Clock: Accelerometer (High Res)\n');
elseif dt_vel <= dt_temp
    t = t_vel_raw;
    fprintf('>> Master Clock: Velocity\n');
else
    t = t_temp_raw;
    fprintf('>> Master Clock: Temperature\n');
end

%% 2. Interpolate ALL Variables to Master Clock
% Temp
temp = interp1(t_temp_raw, temp_raw, t, 'linear', 'extrap');

% Vel
vel_ms_raw = vel_raw / 3.6; 
vel = interp1(t_vel_raw, vel_ms_raw, t, 'linear', 'extrap');

% Accel
accel = interp1(t_accel_raw, accel_raw, t, 'linear', 'extrap');

% Brake Pressure: Interpolate
% FIX IS HERE: Use 'b_p_raw_Pa', NOT 'b_p_raw'
b_p_pa = interp1(t_b_p_raw, b_p_raw_Pa, t, 'linear', 'extrap');

angle_val = interp1(angle_time, angle_val, t, 'linear', 'extrap');

%% 3. Process Acceleration (Bias & Deadzone)
num_samples = length(accel);
tail_start  = round(0.85 * num_samples); 
tail_data   = accel(tail_start:end);

accel_bias = mean(tail_data);
accel = accel - accel_bias; 

noise_floor = max(abs(tail_data - accel_bias));
deadzone_g  = noise_floor * 1; 

fprintf('Calibration:\n  Bias Removed: %.4f g\n  Deadzone Set: %.4f g\n', accel_bias, deadzone_g);

%% 4. Moving Average (Scaled to Sampling Rate)
freq = 9/mean(diff(t));
N = round(1.0 * freq); % 1-second window
if mod(N,2)==0, N=N+1; end 

b = ones(N,1)/N;
a = 1;
y = filter(b,a,temp);
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];

%% 5. Run Simulation Loop

params.accel_deadzone_g = deadzone_g;

T_predicted = zeros(length(t),1);
T_predicted(1) = temp(1); 

brake_threshold = 3000;

for i = 2:length(T_predicted)
    dt_step = t(i) - t(i-1);
    if dt_step <= 0, dt_step = 1e-6; end
    
    dT = delta_Temp(T_predicted(i-1), vel(i), accel(i), b_p_pa(i), params, dt_step, brake_threshold);
    T_predicted(i) = T_predicted(i-1) + dT;
end

%% Fig 1
figure;
scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data', 'MarkerFaceAlpha', 0.2); hold on;
plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average');
plot(t, T_predicted, 'g', 'LineWidth', 2, 'DisplayName', 'Predicted Temp');
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

%% Fig 4 -- Fig 1 + Velocity + Brake Pressure + Accel
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