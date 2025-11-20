function params = buildParams()

    %--------------------
    % Rotor Properties
    %--------------------
    params.m_r      = 0.4354;     % kg
    params.c_r      = 486;        % J/kg-K
    params.A_rotor  = 0.035212;   % m^2
    params.eps      = 0.000319;   % emissivity
    
    % Radiation
    params.sigma    = 5.67e-8;    % W/m2K4
    % CORRECTION: Sheet listed 0.000319 here, but that is likely 
    % the partition coefficient 'p'. Used steel emissivity is ~0.55.
    params.eps      = 0.55;

    %--------------------
    % Air Properties
    %--------------------
    params.rho_air  = 1.17;      % kg/m3
    params.Cp_air   = 1100;      % J/kgK
    params.k_air    = 0.026;     % W/mK
    params.T_amb    = 20;

    %--------------------
    % Friction / Pad Geometry
    %--------------------
    params.mu       = 0.45;      % From 'Redo' tab
    params.A_pis    = 0.0006387; % m2

    % Partition coefficients
    params.e_rotor  = 12926;  
    params.e_pad    = 7500;   % Estimate for pad

    params.S_r      = 0.0010305; % Rotor contact surface
    params.S_p      = 0.0012903; % Pad contact surface

    %--------------------
    % Empirical h(v) Data
    %--------------------
    params.h_vel   = [0 20 40 60 80 100];
params.h_Wm2K  = [9.88 19.9 29.9 39.9 49.9 59.9];

    % HUB SINK FACTOR (NEW)
    % Steel rotors lose heat to the aluminum hat/hub via conduction.
    % We assume 90% of partitioned heat stays in the ring, 10% leaks to hub.
    params.hub_efficiency = 0.95;

    % Vehicle Dynamics
    params.m_car = 227 + 70; % car + driver
    params.bias_front = 0.63;
    
    % Drag
    params.Cd = 1.25;       % Drag coefficient (typical FSAE: 1.0 - 1.5)
    params.A_frontal = 1.132; % Frontal Area in m^2
    params.Crr = 0.015;    % Rolling resistance coefficient
    params.g = 9.81;

    % ENGINE BRAKING (NEW)
    % FSAE engines provide significant drag (approx 0.15g) when off-throttle.
    % This removes load from the brakes.
    params.engine_braking_g = 0.01;

    params.eps = 0.55;

    %params.K_hyd = 0.00014;

end
