clear; close all; clc;

%% 1. Load Data
filename = 'S1_#31435_20251109_162322.mat';
if ~isfile(filename), error('File not found'); end
S1 = load(filename);

% Load Channels
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t_all = B_T.Time(:);
temp_all = B_T.Value(:);
vel_all = S1.Vehicle_Speed.Value(:) / 3.6; % m/s

% LOAD PRESSURE (The Key to Robustness)
% We need to know when brakes are OFF to measure cooling
P_F_chan = S1.Brake_Pressure_Front_Sensor;
t_pres = P_F_chan.Time(:);
p_psi = P_F_chan.Value(:);
p_pa = p_psi * 6894.76; % Convert to Pa

% Interpolate Pressure to match Temp Time
pres_all = interp1(t_pres, p_pa, t_all, 'linear', 'extrap');

%% 2. Define System Constants (Matches buildParams)
params = buildParams(); 
m_rotor = params.m_r;    
c_rotor = params.c_r;       
A_total = params.A_rotor;  
eps     = params.eps;      
sigma   = params.sigma;
T_amb   = params.T_amb;        

%% 3. Calculate Derivatives
% Smooth the temperature to clean up the derivative
% A 2-second window (assuming ~20Hz) helps remove sensor steps
temp_smooth = smoothdata(temp_all, 'gaussian', 40); 
dT_dt = gradient(temp_smooth) ./ gradient(t_all);

%% 4. FILTER: Find "Pure Cooling" Zones (Whole File)
% We use the Pressure Sensor to guarantee brakes are OFF.
% Conditions:
% 1. Brakes OFF (Pressure < 50,000 Pa / ~7 Psi)
% 2. Moving (> 10 km/h) for airflow
% 3. Hot (> 50 C) for signal-to-noise
% 4. Cooling (dT/dt < -0.1)

mask = (pres_all < 50000) ...       % Brakes definitely OFF
     & (vel_all > 2.7) ...          % Speed > 10 km/h
     & (temp_all > 50) ...          % Temp > 50 C
     & (dT_dt < -0.1);              % Actually cooling

v_clean = vel_all(mask);
T_clean = temp_all(mask);
dT_clean = dT_dt(mask);

%% 5. Solve for h
% Energy Balance: -m*c*dT/dt = h*A*(T-Tamb) + Radiation
Q_loss_total = -m_rotor * c_rotor * dT_clean;

T_K = T_clean + 273.15;
T_amb_K = T_amb + 273.15;
Q_rad = eps * sigma * A_total * (T_K.^4 - T_amb_K.^4);

Q_conv = Q_loss_total - Q_rad;
h_calculated = Q_conv ./ (A_total * (T_clean - T_amb));

%% 6. Statistical Binning (Removes Noise)
% Instead of a raw scatter, we bin by speed to find the clean trend
edges = 0:5:120; % Speed buckets (km/h)
v_kmh_clean = v_clean * 3.6;

v_bins = [];
h_medians = [];

for i = 1:length(edges)-1
    bin_mask = (v_kmh_clean >= edges(i)) & (v_kmh_clean < edges(i+1));
    if sum(bin_mask) > 5
        v_bins(end+1) = mean([edges(i), edges(i+1)]); %#ok<SAGROW>
        h_medians(end+1) = median(h_calculated(bin_mask)); %#ok<SAGROW>
    end
end

if isempty(h_medians)
    error('No valid cooling data found. Try lowering Temp threshold to 40.');
end

%% 7. Fit and Plot
p = polyfit(v_bins, h_medians, 1);
v_query = 0:10:120;
h_fit = polyval(p, v_query);
h_fit = max(h_fit, 2); 

figure('Color','w');
scatter(v_kmh_clean, h_calculated, 10, [0.8 0.8 0.8], 'filled'); hold on; % Raw data
plot(v_bins, h_medians, 'bo', 'MarkerFaceColor', 'b'); % Binned Medians
plot(v_query, h_fit, 'r-', 'LineWidth', 3); % Fit

title(['Derived Cooling: h = ' num2str(p(2),'%.1f') ' + ' num2str(p(1),'%.2f') 'V']);
xlabel('Speed (km/h)'); ylabel('h (W/m^2K)');
legend('Raw Noise', 'Speed Bins', 'Fit');
grid on; ylim([0 150]);

%% 8. Output
fprintf('\n------------------------------------------------\n');
fprintf('       NEW COOLING PARAMETERS (WHOLE FILE)      \n');
fprintf('------------------------------------------------\n');
fprintf('params.h_vel   = %s;\n', mat2str(v_query));
fprintf('params.h_Wm2K  = %s;\n', mat2str(h_fit, 3));
fprintf('------------------------------------------------\n');