function params = buildParams()

    %--------------------
    % Rotor Properties
    %--------------------
    params.m_r      = 0.4354;     % kg
    params.c_r      = 523;        % J/kg-K
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
    params.e_pad    = 2500;   % Estimate for pad

    params.S_r      = 0.0010305; % Rotor contact surface
    params.S_p      = 0.0012903; % Pad contact surface

    %--------------------
    % Empirical h(v) Data
    %--------------------
    params.h_vel   = [0 20 40 60];        % km/h
    params.h_Wm2K  = [1.0 21.3 36.0 49.2]; % W/m2K

    % Car
    params.m_car = 227 + 70; % car + driver
    
    % Drag
    params.Cd = 1.25;       % Drag coefficient (typical FSAE: 1.0 - 1.5)
    params.A_frontal = 1.132; % Frontal Area in m^2
    params.Crr = 0.015;    % Rolling resistance coefficient
    params.g = 9.81;

    params.bias_front = 0.63;

    params.eps = 0.55;

end
