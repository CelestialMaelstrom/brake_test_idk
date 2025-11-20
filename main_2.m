clear; close all; clc;

%% 1. Load & Analyze Sampling Rates
S1 = load('S1_#31435_20251109_162322.mat');

% Load Raw Channels
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t_temp_raw = B_T.Time(:);
temp_raw   = B_T.Value(:);

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
% We define 't' as the high-resolution time vector
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
% This ensures t, temp, vel, and accel are all the exact same length
% so your plotting code works without modification.

% Temp: Linear interpolation (draws lines between dots)
temp = interp1(t_temp_raw, temp_raw, t, 'linear', 'extrap');

% Vel: Convert km/h -> m/s and interpolate
vel_ms_raw = vel_raw / 3.6; 
vel = interp1(t_vel_raw, vel_ms_raw, t, 'linear', 'extrap');

% Accel: Interpolate
accel = interp1(t_accel_raw, accel_raw, t, 'linear', 'extrap');

%% 3. Process Acceleration (Bias & Deadzone)
% Use the last 15% of the run to find the "zero" point
num_samples = length(accel);
tail_start  = round(0.85 * num_samples); 
tail_data   = accel(tail_start:end);

accel_bias = mean(tail_data);
accel = accel - accel_bias; % Apply Correction

% Calculate Noise Floor from the tail
noise_floor = max(abs(tail_data - accel_bias));
deadzone_g  = noise_floor * 1.5; % 1.5x Safety Margin

fprintf('Calibration:\n  Bias Removed: %.4f g\n  Deadzone Set: %.4f g\n', accel_bias, deadzone_g);

%% 4. Moving Average (Scaled to Sampling Rate)
% Original code used N=91. If we are now running 10x faster, 
% we should increase N to keep the same smoothing window.
% Assuming original was ~10Hz (N=91 -> ~9s window? Or 0.9s?)
% We will stick to a ~1.0 second smoothing window.
freq = 1/mean(diff(t));
N = round(3.0 * freq); % 1-second window
if mod(N,2)==0, N=N+1; end % Ensure odd number

b = ones(N,1)/N;
a = 1;
y = filter(b,a,temp);
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];

%% 5. Run Simulation Loop
params = buildParams();
params.accel_deadzone_g = deadzone_g;

T_predicted = zeros(length(t),1);
T_predicted(1) = temp(1); % Initialize from data

for i = 2:length(T_predicted)
    dt_step = t(i) - t(i-1);
    
    % Sanity check for duplicate timestamps
    if dt_step <= 0, dt_step = 1e-6; end
    
    dT = delta_Temp(T_predicted(i-1), vel(i), accel(i), params, dt_step);
    T_predicted(i) = T_predicted(i-1) + dT;
end

% ---------------------------------------------------------
% STOP COPYING HERE. 
% Your existing "%% Fig 1" plotting code goes below this line.
% ---------------------------------------------------------


%% Fig 1
    figure;
    scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data', 'MarkerFaceAlpha', 0.2); hold on;
    plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average (delay corrected');
    
    xlabel('Time (s)');
    ylabel('Brake Temperature');
    legend();
    grid on;

    plot(t, T_predicted, 'g', 'LineWidth', 2, 'DisplayName', 'Predicted Temp');

%% Fig 2 - accelerometer
% Plot accelerometer data
figure;
plot(t, accel, 'b', 'LineWidth', 1.5, 'DisplayName', 'Interpolated Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend();
grid on;