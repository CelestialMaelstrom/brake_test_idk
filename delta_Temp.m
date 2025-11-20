function dT = delta_Temp(T_bulk, vel, accel, pressure_Pa, params, dt, brake_threshold)

    % 1. Calculate Forces & Power
    F_inertial = params.m_car * (-accel);
    F_aero = 0.5 * params.rho_air * params.Cd * params.A_frontal * (vel^2);
    F_roll = params.Crr * params.m_car * params.g;
    F_brake_net = F_inertial - F_aero - F_roll;

    %brake_threshold = 100000; % 100kPa (~14.5 Psi) gate
    
    if pressure_Pa > brake_threshold && F_brake_net > 0
        P_car = F_brake_net * vel;
        P_rotor = (P_car * params.bias_front / 2);
        
        % Partition (Sintered Pads absorb heat)
        num = params.e_rotor * params.S_r;
        den = (params.e_rotor * params.S_r) + (params.e_pad * params.S_p);
        H_in = (num / den) * P_rotor;
    else
        H_in = 0;
    end

    % 2. CALCULATE SKIN TEMP (The Physics Fix)
    % Heat Flux = Power / Area
    q_flux = H_in / params.A_rotor;
    
    % Thermal Resistance of the "Skin" (Approximation)
    % T_skin = T_bulk + (Flux * Thickness_Conductivity_Factor)
    % For steel rotors, a factor of ~0.0001 to 0.0003 works well.
    % This represents the temp gradient from surface to core.
    R_th_skin = 0.0002; 
    T_skin = T_bulk + (q_flux * R_th_skin);

    % 3. Cooling (Driven by SKIN Temp, not Bulk)
    v_kmh = max(vel * 3.6, 0);
    h = interp1(params.h_vel, params.h_Wm2K, v_kmh, 'linear', 'extrap');
    
    % Convection & Radiation use T_skin
    % This creates MUCH higher cooling during braking, killing the "Spike"
    H_conv = h * params.A_rotor * (T_skin - params.T_amb);

    T_K = T_skin + 273.15;
    T_amb_K = params.T_amb + 273.15;
    H_rad = params.eps * params.sigma * params.A_rotor * (T_K^4 - T_amb_K^4);

    % 4. Update Bulk Temp (Conservation of Energy)
    % The bulk mass still stores the net energy
    dT_dt = (H_in - H_conv - H_rad) / (params.m_r * params.c_r);
    dT = dT_dt * dt;

end