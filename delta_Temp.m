function dT = delta_Temp(T_current, vel, accel, pressure, params, dt)

    % 1. Calculate Forces
    F_inertial = params.m_car * (-accel);
    F_aero = 0.5 * params.rho_air * params.Cd * params.A_frontal * (vel^2);
    F_roll = params.Crr * params.m_car * params.g;
    
    F_brake_net = F_inertial - F_aero - F_roll;

    % 2. PRESSURE GATING
    % Threshold: 100,000 Pa (~14.5 Psi)
    % Your noise is ~70,000 Pa (10 Psi). This clears it safely.
    brake_threshold = 100000;
    
    if pressure > brake_threshold && F_brake_net > 0
        % Brakes are PRESSED and Deceleration is occurring
        
        P_car_total = F_brake_net * vel; 
        
        % Split Logic
        P_front_axle = P_car_total * params.bias_front;
        P_rotor_input = P_front_axle / 2;
        
        % Partition
        num = params.e_rotor * params.S_r;
        den = (params.e_rotor * params.S_r) + (params.e_pad * params.S_p);
        p = num / den;
        
        H_d = p * P_rotor_input;
        
    else
        H_d = 0;
    end

    % 3. Cooling
    v_kmh = max(vel * 3.6, 0);
    h = interp1(params.h_vel, params.h_Wm2K, v_kmh, 'linear', 'extrap');
    H_conv = h * params.A_rotor * (T_current - params.T_amb);

    T_K = T_current + 273.15;
    T_amb_K = params.T_amb + 273.15;
    H_rad = params.eps * params.sigma * params.A_rotor * (T_K^4 - T_amb_K^4);

    % 4. Total Change
    dT_dt = (H_d - H_conv - H_rad) / (params.m_r * params.c_r);
    dT = dT_dt * dt;

end