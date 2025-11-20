function h_static = auto_tune_cooling(S1, params)
    % AUTO_TUNE_COOLING Finds stationary cool-down periods to calibrate h.
    
    % 1. Extract Data
    B_T = S1.Brake_Temperature_Front_Right_Sensor;
    t = B_T.Time(:);
    T = B_T.Value(:);
    v = S1.Vehicle_Speed.Value(:) / 3.6; % m/s
    
    % 2. Find "Stationary & Cooling" Zones
    % Rules: Speed < 0.5 m/s, Temp > 50C, and Cooling Down
    mask = (v < 0.5) & (T > 50);
    
    % Find the longest contiguous segment to ensure a good fit
    diff_mask = diff([0; mask; 0]);
    start_idxs = find(diff_mask == 1);
    end_idxs = find(diff_mask == -1) - 1;
    
    if isempty(start_idxs)
        warning('No stationary cooling data found (>50C). Defaulting h_static=5.');
        h_static = 5.0;
        return;
    end
    
    [dur, best_idx] = max(end_idxs - start_idxs);
    
    % If the segment is too short (e.g., < 10 seconds), reliability is low
    if dur < 50 
        warning('Stationary data too short for fit. Defaulting h_static=5.');
        h_static = 5.0;
        return;
    end
    
    % Extract the segment
    range = start_idxs(best_idx):end_idxs(best_idx);
    t_seg = t(range) - t(range(1)); % Time starting at 0
    T_seg = T(range);
    
    % 3. Fit Exponential Decay: T(t) = Tamb + (Tstart - Tamb)*e^(-t/tau)
    % Linearize: ln(T - Tamb) = - (1/tau)*t + C
    y_data = log(T_seg - params.T_amb);
    
    % Robust Fit (Polyfit degree 1)
    p = polyfit(t_seg, y_data, 1);
    tau = -1 / p(1); % Time Constant
    
    % 4. Calculate h
    % tau = (m*c) / (h*A)  ->  h = (m*c) / (tau*A)
    h_static = (params.m_r * params.c_r) / (tau * params.A_rotor);
    
    fprintf('  [Auto-Tune] Found %d sec stationary window.\n', round(max(t_seg)));
    fprintf('  [Auto-Tune] Measured Time Constant (tau): %.1f s\n', tau);
    fprintf('  [Auto-Tune] Calculated Baseline h (v=0):  %.2f W/m2K\n', h_static);

end