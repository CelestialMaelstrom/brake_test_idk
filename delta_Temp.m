function dT = delta_Temp(T_current, vel, accel, params, dt)

    % 1. Calculate Forces
    F_inertial = params.m_car * (-accel);
    F_aero = 0.5 * params.rho_air * params.Cd * params.A_frontal * (vel^2);
    F_roll = params.Crr * params.m_car * params.g;
    
    % 2. Apply Deadzone
    F_brake_raw = F_inertial - F_aero - F_roll;

    % Convert g-force deadzone to Newtons: F = m * a
    % We default to 0.05g if the param doesn't exist yet (safety check)
    if isfield(params, 'accel_deadzone_g')
        deadzone_g = params.accel_deadzone_g;
    else
        deadzone_g = 0.05; 
    end
    
    F_deadzone_N = deadzone_g * 9.81 * params.m_car;

    % Filter the force
    if F_brake_raw < F_deadzone_N
        F_brake_net = 0;
    else
        F_brake_net = F_brake_raw;
    end

    %-----------------------------------
    % 3. Brake Power Input
    %-----------------------------------
    if F_brake_net > 0
        % Total braking power of the whole car
        P_car_total = F_brake_net * vel; 
        
        % APPLY BIAS: 
        % params.bias_front (0.63) goes to Front Axle
        P_front_axle = P_car_total * params.bias_front;
        
        % SPLIT L/R: 
        % 50% of axle power goes to THIS single rotor
        P_rotor_input = P_front_axle / 2;
        
        % PARTITION: Heat split between Rotor and Pad
        % FIX: Use distinct effusivity for Rotor (Steel) vs Pad
        % Formula: p = (e_r * S_r) / (e_r * S_r + e_p * S_p)
        numerator = params.e_rotor * params.S_r;
        denominator = (params.e_rotor * params.S_r) + (params.e_pad * params.S_p);
        
        p = numerator / denominator;
        
        H_d = p * P_rotor_input;
        
    else
        % Car is accelerating or coasting (drag > inertial force)
        H_d = 0;
    end

    %-----------------------------------
    % 4. Cooling (Convection + Radiation)
    %-----------------------------------
    % Convection (using your lookup table)
    v_kmh = max(vel * 3.6, 0);
    h = interp1(params.h_vel, params.h_Wm2K, v_kmh, 'linear', 'extrap');
    H_conv = h * params.A_rotor * (T_current - params.T_amb);

    % Radiation (Corrected to Kelvin)
    T_K = T_current + 273.15;
    T_amb_K = params.T_amb + 273.15;
    
    % Using corrected emissivity (0.55) from buildParams
    H_rad = params.eps * params.sigma * params.A_rotor * (T_K^4 - T_amb_K^4);

    %-----------------------------------
    % 5. Calculate dT
    %-----------------------------------
    dT_dt = (H_d - H_conv - H_rad) / (params.m_r * params.c_r);
    dT = dT_dt * dt;

end