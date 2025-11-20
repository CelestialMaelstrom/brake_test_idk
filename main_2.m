clear; close all; clc;

%% Build params
params = buildParams();

%% Gather channels from S1 file
S1 = load('S1_#31435_20251109_162322.mat'); % Load the S1 file

%% Brake Temp
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t = B_T.Time;
temp = B_T.Value;
t = t(:);
temp = temp(:);

%% Velocity
vel_raw = S1.Vehicle_Speed.Value;
vel = vel_raw / 3.6; % CONVERSION: km/h -> m/s
vel = vel(:);

%% Acceleration
%% Acceleration Processing & Calibration
accel_raw = S1.IRIMU_V2_IMU_Acceleration_Y_Axis.Value;
accel_time = S1.IRIMU_V2_IMU_Acceleration_Y_Axis.Time;

% 1. Interpolate to match Temperature time vector
accel_interp = interp1(accel_time, accel_raw, t, 'linear', 'extrap');

% 2. Auto-Calibration (Zeroing)
% Define "Tail End" as the last 15% of the dataset
num_samples = length(accel_interp);
tail_start = round(0.85 * num_samples); 
tail_data = accel_interp(tail_start:end);

% Calculate the Bias (Average of the tail)
accel_bias = mean(tail_data);

% Apply correction to the ENTIRE dataset
accel_corrected = accel_interp - accel_bias;
accel = accel_corrected;

% 3. Determine Deadzone (Noise Floor)
% Find the maximum noise spike in the tail section
noise_floor = max(abs(tail_data - accel_bias));

% Set deadzone to 1.5x the noise floor to be safe
params.accel_deadzone_g = noise_floor * 1;

fprintf('Calibration Complete:\n');
fprintf('  Bias Removed: %.4f g\n', accel_bias);
fprintf('  Deadzone Set: %.4f g\n', params.accel_deadzone_g);



% Moving average window (odd number recommended)
N = 91;                      
b = ones(N,1)/N;
a = 1;
% Apply moving average filter
y = filter(b,a,temp);
% Delay compensation
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];


%% Predict Temp


% For each timestep of original data (t), we call delta_Temp and 'integrate' it
T_predicted = zeros(length(t),1);
T_predicted(1) = temp(1); % Start of data
for i = 2:length(T_predicted)
    dt_step = t(i) - t(i-1);
    % Pass the PREVIOUS temperature to calculate the CURRENT change
    dT = delta_Temp(T_predicted(i-1), vel(i), accel(i), params, dt_step);
    T_predicted(i) = T_predicted(i-1) + dT;
end


%% Fig 1
    figure;
    scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data'); hold on;
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