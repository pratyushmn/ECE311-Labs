% Data plotting for ECE311/ECE356 lab 3.
% Last modified March 8, 2016.

clear all
close all
clc
delete(instrfindall);


%% PARAMETERS (CAN BE CHANGED)

% Variables created by this experiment will be saved in a data file
% called [save_file].mat, where [save_file] is specified below.
% Variables saved:
%    position        vector of position samples (in meters)
%    n_samples       number of samples plotted for all data
%    save_file       file containing saved experiment data
%    stop_time       time the experiment is stopped (in seconds)
%    T_plot          sample time for all plotted data (in seconds)
%    time            vector of samples times (in seconds)
%    velocity        vector of raw, unfiltered velocity samples (in m/s)
%    velocity_filt   vector of filtered velocity samples (in m/s)
%    velocity_ref    vector of reference velocity samples (in m/s)
save_file = 'Lab3';

% Ensure the following serial port is set to the same serial port being
% used by the Arduino controller.
serial_port = 'COM4';


%% DO NOT EDIT ANYTHING BELOW THIS POINT -------------------------------->

% Velocity source
VELOCITY_SOURCE = 'observer'; % either 'observer' or 'diffpos'
assert(any(strcmp(VELOCITY_SOURCE, {'observer','diffpos'})));

% Filtering technique
FILTER_TYPE = 'median+butter'; % either 'median+butter' or 'butter'
assert(any(strcmp(FILTER_TYPE, {'median+butter','butter'})));

% Flag: include +/-2% bands in plot
INCLUDE_2PCT_BANDS = true; % either true or false
assert(islogical(INCLUDE_2PCT_BANDS));

% Set the time span and interval for data collection
stop_time = 15;
T_plot = 0.01;
n_samples = fix(stop_time/T_plot);
encoder_to_pos = 2.2749*0.00001;


%% Create the serial object
serial_object = serial(serial_port, 'BaudRate', 28800);
fopen(serial_object);


%% Set up the figure window
figure_handle = figure('NumberTitle','off', 'Name','Tracking');

% Set axes
axes_handle = axes('Parent',figure_handle, 'YGrid','on', 'XGrid','on');
hold on

plot_handle = plot(axes_handle, 0,0, 'Marker','.', 'LineWidth',1);
xlim(axes_handle, [0 stop_time]);
ylim(axes_handle, [-1 1]);

% Create xlabel
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);

% Create ylabel
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);

% Create title
title('Cart Variables as a Function of Time on a Tilted Track', ...
    'FontWeight','bold', 'FontSize',15);


%% Collect data
time         = T_plot*(0:n_samples-1);
position     = zeros(1, n_samples);
velocity_ref = zeros(1, n_samples);
velocity_obs = zeros(1, n_samples);

for i = 2:n_samples
    % collect the next sample
    position(i) = fscanf(serial_object, '%f')*encoder_to_pos;
    velocity_ref(i) = fscanf(serial_object, '%f')/1000;
    velocity_obs(i) = fscanf(serial_object, '%f')/1000;

    % update the plot
    set(plot_handle, 'YData',position(1:i), 'XData',time(1:i));
    set(figure_handle, 'Visible','on');
end


%% Compute the velocity and filtered velocity
if strcmp( VELOCITY_SOURCE, 'diffpos' )
    % Compute velocity by differentiating position
    position_pp = spline(time, position);
    velocity_pp = fnder(position_pp);
    velocity = ppval(velocity_pp, time);
else
    % Compute velocity from the observer state reported by the Arduino
    % controller
    velocity = velocity_obs;
end

if strcmp( FILTER_TYPE, 'butter' )
    % Smooth velocity data using an aggressive Butterworth LP filter
    [b,a] = butter(10, 0.25);
    velocity_filt = filtfilt(b, a, velocity);
else
    % Smooth velocity data using a median filter followed by a less
    % aggressive Butterworth LP filter
    velocity_med = zeros(1, n_samples);
    v_m_window = 9;
    for k = 1:n_samples
        velocity_med(k) = median(...
            velocity(max(1, k-v_m_window):min(n_samples, k+v_m_window)));
    end
    [b,a] = butter(6, 0.45, 'low');
    velocity_filt = filtfilt(b, a, velocity_med);
end


%% Plot all outstanding data
if INCLUDE_2PCT_BANDS
    % Plot +/-2% bands if flagged
    reference = max(velocity_ref);
    time_ival = [0 stop_time];
    ref_ival = [1 1]*reference;
    band_ival = 0.02*(reference - (-reference));

    plot(time_ival, ref_ival+band_ival, 'b', ...
        time_ival,  ref_ival-band_ival, 'b', ...
        time_ival, -ref_ival+band_ival, 'b', ...
        time_ival, -ref_ival-band_ival, 'b', 'LineWidth', 1);
end

% Plot position, velocity, reference velocity, and filtered velocity
hline1 = plot(time, position, 'b', 'Linewidth',1);
hline2 = plot(time, velocity, 'Linewidth',1, 'Color',[0.6,0.6,0.6]); 
hline3 = plot(time, velocity_ref, 'k', 'Linewidth',2);
hline4 = plot(time, velocity_filt, 'r', 'Linewidth',2);

xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14)
ylabel('Position [m]/Velocity [m/s]', 'FontWeight','bold', 'FontSize',14);
legend([hline1,hline2,hline3,hline4], 'Position', 'Velocity', ...
    'Reference Velocity', 'Filtered Velocity');

hold off


%% Close the serial object
fclose(serial_object);


%% Clean up temporary variables and save
clear serial_port serial_object figure_handle axes_handle plot_handle
clear encoder_to_pos velocity_obs i
clear b a position_pp velocity_pp
clear b a k reference velocity_med v_m_window
clear time_ival ref_ival band_ival
clear hline1 hline2 hline3 hline4
clear VELOCITY_SOURCE FILTER_TYPE INCLUDE_2PCT_BANDS

save(save_file);
