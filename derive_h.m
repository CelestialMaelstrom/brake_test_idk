clear; close all; clc;

%% 1. Load Data
filename = 'S1_#31435_20251109_162322.mat';
if isfile(filename)
    S1 = load(filename);
else
    error('File not found. Check filename.');
end

% Load Channels
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t = B_T.Time(:);
temp = B_T.Value(:);

% Speed (Convert to m/s)
v_raw = S1.Vehicle_Speed.Value(:);
vel = v_raw / 3.6; 

% Acceleration (Y-Axis as per your update)
accel_raw = S1.IRIMU_V2_IMU_Acceleration_Y_Axis.Value(:);
accel_t = S1.IRIMU_V2_IMU_Acceleration_Y_Axis.Time;
% Interpolate accel to match temp timing
accel = interp1(accel_t, accel_raw, t, 'linear', 'extrap');

%% 2. Define System Constants (Must match buildParams)
% These must be accurate for the math to hold
m_rotor = 0.4354;    % <--- UPDATE THIS (Was 0.3817)
c_rotor = 486;       % Matches your buildParams
A_total = 0.035212;  % Matches your buildParams
eps     = 0.55;      % Emissivity
sigma   = 5.67e-8;
T_amb   = 20;        % Assumed ambient C

%% 3. Calculate Derivatives
% Smooth the temperature slightly to reduce sensor noise derivatives
temp_smooth = smoothdata(temp, 'gaussian', 20); 
dT_dt = gradient(temp_smooth) ./ gradient(t);

%% 4. Filter: Find "Pure Cooling" Zones
% We want points where:
% 1. Car is moving fast enough for clean air (> 10 km/h)
% 2. Brakes are DEFINITELY OFF (Accel > -0.1g means Coasting or Throttle)
% 3. Rotor is hot enough to radiate/convect clearly (> 80 C)
% 4. Temperature is actually dropping (dT/dt < 0)

mask = (vel > 3) ...              % > 10 km/h
     & (accel > -0.1 * 9.81) ...  % No significant deceleration (Brakes off)
     & (temp > 80) ...            % Signal-to-noise ratio is poor below 80C
     & (dT_dt < -0.2);            % Must be cooling down

% Extract clean data
v_clean = vel(mask);
T_clean = temp(mask);
dT_clean = dT_dt(mask);

%% 5. Solve for h
% Energy Balance: Q_stored_loss = Q_conv + Q_rad
% -m*c*dT/dt = h*A*(T-Tamb) + sigma*eps*A*(T^4 - Tamb^4)

% Total Heat Loss Rate (Watts)
Q_loss_total = -m_rotor * c_rotor * dT_clean;

% Radiation Component (Watts)
T_K = T_clean + 273.15;
T_amb_K = T_amb + 273.15;
Q_rad = eps * sigma * A_total * (T_K.^4 - T_amb_K.^4);

% Convection Component (Watts)
Q_conv = Q_loss_total - Q_rad;

% Solve h (W/m^2K)
h_calculated = Q_conv ./ (A_total * (T_clean - T_amb));

%% 6. Fit and Plot
if isempty(h_calculated)
    error('No valid cooling data found. Try lowering the temp threshold or checking units.');
end

% Convert speed back to km/h for the plot/table
v_kmh_clean = v_clean * 3.6;

% Linear Fit (h = m*v + c)
p = polyfit(v_kmh_clean, h_calculated, 1);
v_query = 0:10:120;
h_fit = polyval(p, v_query);

% Enforce h >= 0 (Physics check)
h_fit = max(h_fit, 0);

% Plot
figure('Color','w');
scatter(v_kmh_clean, h_calculated, 15, 'k', 'filled', 'MarkerFaceAlpha', 0.8); hold on;
plot(v_query, h_fit, 'r-', 'LineWidth', 3);
title('Derived Convection Coefficient (h) vs Speed');
xlabel('Speed (km/h)');
ylabel('h (W/m^2K)');
legend('Measured Points', 'Linear Fit', 'Location','northwest');
grid on;
ylim([0 150]);

%% 7. Output for buildParams.m
fprintf('------------------------------------------------\n');
fprintf('       NEW COOLING PARAMETERS FOUND             \n');
fprintf('------------------------------------------------\n');
fprintf('Paste this into buildParams.m:\n\n');
fprintf('params.h_vel   = %s;\n', mat2str(v_query));
fprintf('params.h_Wm2K  = %s;\n', mat2str(h_fit, 3));
fprintf('------------------------------------------------\n');