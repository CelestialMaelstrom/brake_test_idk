clear; close all; clc;

% Gather channels from S1 file
S1 = load('S1_#31435_20251109_162322.mat'); % Load the S1 file

%% Brake Temp
B_T = S1.Brake_Temperature_Front_Right_Sensor;
t = B_T.Time;
temp = B_T.Value;
t = t(:);
temp = temp(:);

% Moving average window (odd number recommended)
N = 91;                      
b = ones(N,1)/N;
a = 1;
% Apply moving average filter
y = filter(b,a,temp);
% Delay compensation
delay = floor((N-1)/2);
y_shifted = [y(delay+1:end); NaN(delay,1)];

% Envelopes
% [yHigh, yLow] = envelope(x,10,"peak");

%% Brake Pressure Front Sensor
B_P_F = S1.Brake_Pressure_Front_Sensor;
t2 = B_P_F.Time;
bpf = B_P_F.Value;
t2 = t2(:);
bpf = bpf(:);

% normalize max to 1
bpf = bpf / max(bpf); % Normalize brake pressure front sensor values

%% Brake Pressure Rear Sensor
B_P_R = S1.Brake_Pressure_Rear_Sensor;
bpr = B_P_R.Value;
bpr = bpr(:);

% Normalize rear brake pressure values
bpr = bpr / max(bpr); % Normalize brake pressure rear sensor values

%% Brake Pressure Avg
bp_avg = (bpf + bpr)/2;

%% Braking Events
samp_dt_2 = mean(diff(t2));
%
minSamples = round(2 / samp_dt_2); % Can't be within 2 secs of each other
[b_events_peaks, b_events_locs] = findpeaks(bp_avg, 'MinPeakProminence', 0.1, 'MinPeakDistance', minSamples);

%% Plotting
    %% Fig 1
    figure;
    scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data'); hold on;
    plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average (delay corrected');
    
    xlabel('Time (s)');
    ylabel('Brake Temperature');
    legend();
    grid on;

    %% Fig 2
    figure();
    plot(t2, bpf, 'DisplayName', 'Brake Pressure Front Sensor'); hold on;
    plot(t2, bpr, 'DisplayName', 'Brake Pressure Rear Sensor'); hold off;

    xlabel('Time (s)');
    ylabel('Brake Pressure, Normalized');
    legend();
    ylim([min(bpr)-0.01, max(bpr)+0.01]);

    %% Fig 3
    figure();
    plot(t2, bp_avg, 'LineWidth', 0.5, 'DisplayName', 'Brake Pressure Average'); hold on;

    % plot peaks
    scatter(t2(b_events_locs), b_events_peaks, 150, 'x', 'MarkerEdgeColor', 'k', 'DisplayName', 'Braking Events');
    hold off;

    xlabel('Time (s)');
    ylabel('Brake Pressure, Normalized');
    legend();
    ylim([min(bpr)-0.01, max(bpr)+0.01]);

    %% Fig 4
    % Plot Fig 1 and Fig 2 together via subplots
    figure();
    subplot(2,1,1);
    scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data'); hold on;
    plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average (delay corrected');
    
    xlabel('Time (s)');
    ylabel('Brake Temperature');
    legend();
    grid on;

    subplot(2,1,2);
    plot(t2, bp_avg, 'LineWidth', 0.5, 'DisplayName', 'Brake Pressure Average'); hold on;

    % plot peaks
    scatter(t2(b_events_locs), b_events_peaks, 150, 'x', 'MarkerEdgeColor', 'k', 'DisplayName', 'Braking Events');
    hold off;

    xlabel('Time (s)');
    ylabel('Brake Pressure, Normalized');
    legend();
    ylim([min(bpr)-0.01, max(bpr)+0.01]);


    %% Fig 5
    % Plot Fig 4 as one with xlines
    figure();
    scatter(t, temp, 1, 'filled', 'DisplayName', 'Raw Data'); hold on;
    plot(t, y_shifted, 'r', 'LineWidth', 2, 'DisplayName', 'Moving Average (delay corrected');
    xline(t2(b_events_locs), 'HandleVisibility', 'off');
    xline(t2(b_events_locs(1)), 'DisplayName', 'Braking Events');

    xlabel('Time (s)');
    ylabel('Brake Temperature');
    legend();
    grid on;


    


