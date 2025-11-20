function [dT, F_brake_at_tire] = delta_Temp_P(T_bulk, vel, pressure_Pa, params, dt, brake_threshold)
% DELTA_TEMP_P Calculates temp change based on Hydraulic Pressure.

    % 1. Calculate Braking Force (The "Truth" from the sensor)
    % K_hyd lumps together Piston Area, Friction Coeff, and Radius Ratios.
    % F_brake_at_tire is the total decel force provided by the Front Axle (if sensor is Front)
    
    % Threshold: 100 kPa (~15 Psi) to ignore noise
    if pressure_Pa > brake_threshold
        F_brake_at_tire = pressure_Pa * params.K_hyd;
    else
        F_brake_at_tire = 0;
    end

    % 2. Calculate Heat Input
    % Power = Force * Velocity
    P_total_axle = F_brake_at_tire * vel;
    
    % Split to ONE rotor (Left/Right)
    % Note: We don't need 'bias_front' here because we are measuring 
    % the Front Line Pressure directly!
    P_rotor = P_total_axle / 2; 
    
    % Partition (Sintered Pads)
    num = params.e_rotor * params.S_r;
    den = (params.e_rotor * params.S_r) + (params.e_pad * params.S_p);
    H_in = (num / den) * P_rotor;

    % 3. Solve Skin Temp (2-Node Model)
    q_flux = H_in / params.A_rotor;
    R_th_skin = 0.0002; 
    T_skin = T_bulk + (q_flux * R_th_skin);

    % 4. Cooling (From Skin)
    v_kmh = max(vel * 3.6, 0);
    h = interp1(params.h_vel, params.h_Wm2K, v_kmh, 'linear', 'extrap');
    
    H_conv = h * params.A_rotor * (T_skin - params.T_amb);
    T_K = T_skin + 273.15;
    T_amb_K = params.T_amb + 273.15;
    H_rad = params.eps * params.sigma * params.A_rotor * (T_K^4 - T_amb_K^4);

    % 5. Bulk Temp Change
    dT_dt = (H_in - H_conv - H_rad) / (params.m_r * params.c_r);
    dT = dT_dt * dt;

end