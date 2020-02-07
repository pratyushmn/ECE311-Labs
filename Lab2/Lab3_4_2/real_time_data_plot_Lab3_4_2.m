

%% Create the serial object

clear all
close all
clc

delete(instrfindall);

%% PARAMETERS (CAN BE CHANGED)

save_file = 'Lab3_4_2';


%% PARAMETERS (DO NOT CHANGE)

%% Set the time span and interval for data collection
stopTime = 15;
T_plot = 0.01;
encoder_to_pos = 2.2749*0.00001;



serialPort = 'COM4';
serialObject = serial(serialPort, 'BaudRate', 19200);
fopen(serialObject);



%% Set up the figure window


position = 0;

figureHandle = figure('NumberTitle','off',...
    'Name','Tracking' );
 
% Set axes
axesHandle = axes('Parent',figureHandle,...
    'YGrid','on', 'XGrid','on' );
  

hold on;

plotHandle = plot(axesHandle,0,position,'Marker','.','LineWidth',1);

xlim(axesHandle,[0 stopTime]);
ylim(axesHandle,[-1 1]);

% Create xlabel
xlabel('Time','FontWeight','bold','FontSize',14);

% Create ylabel
%ylabel('Position [m]','FontWeight','bold','FontSize',14);

% Create title
title('Lab3-4-2','FontWeight','bold','FontSize',15);

%plot([0 15],[0.02 0.02],'--k','Linewidth',2);

%% Collect data
count = 2;
time = 0;
position(1) = 0;
%obs(1)=0;
while time(end)<stopTime

    time = [time count*T_plot];
    position(count) = fscanf(serialObject,'%f') ; 
    position(count) = position(count)*encoder_to_pos;
   
    set(plotHandle,'YData',position,'XData',time);

    %obs(count) = fscanf(serialObject,'%f')/1000;
    %velocity(count) = fscanf(serialObject,'%f')/1000;

    set(figureHandle,'Visible','on');

    count = count +1;
    
end
%calculate derivative via spline and differentiation
positionpp = spline(time, position);
velocitypp = fnder(positionpp);
velocity = ppval(velocitypp, time);

%add a filter to get rid of noise
[B,A] = butter(10,0.25);
vel_ave = filtfilt(B,A,velocity);

hline1 = plot(time,position,'b','Linewidth',1);
hline2 = plot(time,velocity,'r','Linewidth',1);
hline3 = plot(time,vel_ave, 'm', 'Linewidth', 2);
xlabel('Time [s]','FontWeight','bold','FontSize',16)
legend([hline1,hline2,hline3],'Position','Velocity','Average Velocity');
hold off

%% Modify file name to save the data
save(save_file);  


%% Clean up the serial object
fclose(serialObject);
%delete(serialObject);
clear serialObject;