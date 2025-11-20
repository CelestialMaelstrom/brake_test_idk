clear; close all; clc;

%% 1. Load & Analyze Sampling Rates
S1 = load('S1_#31435_20251109_162322.mat');

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
b_p = interp1(t_b_p_raw, b_p_raw_Pa, t, 'linear', 'extrap');

%% 3. Process Acceleration (Bias & Deadzone)
num_samples = length(accel);
tail_start  = round(0.85 * num_samples); 
tail_data   = accel(tail_start:end);

accel_bias = mean(tail_data);
accel = accel - accel_bias; 

noise_floor = max(abs(tail_data - accel_bias));
deadzone_g  = noise_floor * 1.5; 

fprintf('Calibration:\n  Bias Removed: %.4f g\n  Deadzone Set: %.4f g\n', accel_bias, deadzone_g);

%% 4. Moving Average (Scaled to Sampling Rate)
freq = 1/mean(diff(t));
N = round(1.0 * freq); % 1-second window
if mod(N,2)==0, N=N+1; end 

b = ones(N,1)/N;
a = 1;
y = filter(b,a,temp);
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];

%% 5. Run Simulation Loop
params = buildParams();
params.accel_deadzone_g = deadzone_g;

T_predicted = zeros(length(t),1);
T_predicted(1) = temp(1); 

for i = 2:length(T_predicted)
    dt_step = t(i) - t(i-1);
    if dt_step <= 0, dt_step = 1e-6; end
    
    dT = delta_Temp(T_predicted(i-1), vel(i), accel(i), b_p(i), params, dt_step);
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
plot(t, b_p, 'm', 'DisplayName', 'Brake Pressure (Pa)');
xlabel('Time (s)'); ylabel('Pressure (Pa)');
grid on;