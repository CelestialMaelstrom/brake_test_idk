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
t_all = B_T.Time(:);
temp_all = B_T.Value(:);
vel_all = S1.Vehicle_Speed.Value(:) / 3.6; % m/s

%% 2. ISOLATE TAIL DATA (The "Last Period")
% We only look at the last 20% of the data to find the cooldown curve
tail_percent = 0.20; 
idx_start = round((1 - tail_percent) * length(t_all));

t = t_all(idx_start:end);
temp = temp_all(idx_start:end);
vel = vel_all(idx_start:end);

%% 3. Define System Constants (Matches buildParams)
params = buildParams(); % Pulls mass/area/material from your central file
m_rotor = params.m_r;    
c_rotor = params.c_r;       
A_total = params.A_rotor;  
eps     = params.eps;      
sigma   = params.sigma;
T_amb   = params.T_amb;        

%% 4. Calculate Derivatives
% Smooth the temperature to clean up the derivative
temp_smooth = smoothdata(temp, 'gaussian', 30); 
dT_dt = gradient(temp_smooth) ./ gradient(t);

%% 5. Filter: Find Valid Cooling Points
% Conditions:
% 1. Moving (> 5 km/h)
% 2. Cooling Down (dT/dt < -0.05)
% 3. Hot enough to matter (> 40C)
mask = (vel > 1.5) & (dT_dt < -0.05) & (temp > 40);

v_clean = vel(mask);
T_clean = temp(mask);
dT_clean = dT_dt(mask);

%% 6. Solve for h
% Energy Balance: -m*c*dT/dt = h*A*(T-Tamb) + Radiation
Q_loss_total = -m_rotor * c_rotor * dT_clean;

T_K = T_clean + 273.15;
T_amb_K = T_amb + 273.15;
Q_rad = eps * sigma * A_total * (T_K.^4 - T_amb_K.^4);

Q_conv = Q_loss_total - Q_rad;
h_calculated = Q_conv ./ (A_total * (T_clean - T_amb));

%% 7. Fit and Plot
if isempty(h_calculated)
    error('No valid cooling data found in the tail section.');
end

v_kmh_clean = v_clean * 3.6;

% Linear Fit
p = polyfit(v_kmh_clean, h_calculated, 1);
v_query = 0:10:120;
h_fit = polyval(p, v_query);
h_fit = max(h_fit, 2); % Clamp to minimum 2 (Natural Convection)

figure('Color','w');
scatter(v_kmh_clean, h_calculated, 20, 'b', 'filled', 'MarkerFaceAlpha', 0.3); hold on;
plot(v_query, h_fit, 'r-', 'LineWidth', 3);
title('Derived Cooling (Tail Section Only)');
xlabel('Speed (km/h)'); ylabel('h (W/m^2K)');
grid on;

%% 8. Output
fprintf('------------------------------------------------\n');
fprintf('       NEW COOLING PARAMETERS (FROM TAIL)       \n');
fprintf('------------------------------------------------\n');
fprintf('params.h_vel   = %s;\n', mat2str(v_query));
fprintf('params.h_Wm2K  = %s;\n', mat2str(h_fit, 3));
fprintf('------------------------------------------------\n');