clear; close all; clc;

%% Gather channels from S1 file
S1 = load('S1_#31435_20251109_162322.mat'); % Load the S1 file

%% Brake Temp
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t = B_T.Time;
temp = B_T.Value;
t = t(:);
temp = temp(:);

%% Velocity
vel = S1.Vehicle_Speed.Value;
vel = vel(:);

%% Acceleration
accel = S1.IRIMU_V2_IMU_Acceleration_X_Axis.Value;
% Convert acceleration sampling rate to that of temperature
accel_time = S1.IRIMU_V2_IMU_Acceleration_X_Axis.Time;
% Interpolate acceleration values to match the temperature time vector
accel_interp = interp1(accel_time, accel, t, 'linear', 'extrap');



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
% Build params
params = buildParams();

% For each timestep of original data (t), we call delta_Temp and 'integrate' it
T_predicted = zeros(length(t),1);
T_predicted(1) = 51.68; % Start of data
timestep = mean(diff(t)); % Calculate the average time step between measurements
for i = 2:length(T_predicted)
    % Pass the PREVIOUS temperature to calculate the CURRENT change
    dT = delta_Temp(T_predicted(i-1), vel(i), accel(i), params, timestep);
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
plot(accel_time, accel, 'b', 'LineWidth', 1.5, 'DisplayName', 'Interpolated Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend();
grid on;