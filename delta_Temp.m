function dT = delta_Temp(T_current, vel, accel, params, dt)

    %-----------------------------------
    % 1. Brake Power Input From Decel
    %-----------------------------------
    if accel < 0
        P_brake = params.m_car * (-accel) * vel;   % watts
    else
        P_brake = 0;
    end
    
    % Fraction to rotor
    p = params.e * params.S_r / (params.e * params.S_r + params.e * params.S_p);
    H_d = p * P_brake;


    %-----------------------------------
    % 2. Convection (use your table)
    %-----------------------------------
    v_kmh = max(vel * 3.6, 0);  % m/s → km/h
    h = interp1(params.h_vel, params.h_Wm2K, v_kmh, 'linear', 'extrap');

    H_conv = h * params.A_rotor * max(T_current - params.T_amb, 0);

    %-----------------------------------
    % 3. Radiation
    %-----------------------------------
    T_K = T_current + 273.15;
    T_amb_K = 293.15;   % 20°C

    H_rad = params.eps * params.sigma * params.A_rotor * (T_K^4 - T_amb_K^4);

    %-----------------------------------
    % 4. Rotor temperature change
    %-----------------------------------
    dT_dt = (H_d - H_conv - H_rad) / (params.m_r * params.c_r);

    dT = dT_dt * dt;

end
