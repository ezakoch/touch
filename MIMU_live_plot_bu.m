%% ADC value liveplotting & logging.
%
% Reads ADC values from the M2 microcontroller.
% Plots the data realtime in a subplot.
% Logged into ADC_Log[time, ADC_Value] variable.
% Hit Ctrl-C to quit the program
%
% By Nick McGill
% Edited by Eza Koch for ADC controller debugging
% Reads m_imu values from the M2 microcontroller and plots the desired data
% in real time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize program and USB port
% Close any existing open port connections
% For the first time running, comment this.
% if(exist('M2USB'))
%     fclose(M2USB);
% else
%     fclose(instrfindall);
% end
% 
% clear all
% close all
% 

%% If the above initialization does not work, please run the following commands manually and disconnect and reconnect USB.
% fclose(serial('COM5','Baudrate', 9600));
% fclose(instrfindall);
 clear all;
 close all;


%% VARIABLES
maxPoints = 20;         % Max number of data points displayed at a time.
t = 1:1:maxPoints;      % Create an evenly spaced matrix for X-axis.
Controller_Log = zeros(2,2);   % Controller data points logged here in format: [time ADC_Value]
Controller_live_plot_log = zeros(maxPoints,1); % Create an array to store ADC values.
Controller_target = zeros(1,maxPoints);
Controller_actual = zeros(1,maxPoints);

IMU_RANGE = -2000:2000;   % signed integer 16-bit
LOGFREQUENCY = 1;  % Log the ADC value every certain number of times.  Lower = more data points.


%% SERIAL
%----> for ***WINDOZE***
% M2USB = serial('COM5','Baudrate', 9600);
% *** Use the device manager to check where the microcontroller is plugged
% into.

%----> for ***MAC***
M2USB = serial('/dev/tty.usbmodem411','Baudrate',9600);
% *** Check where your device is by opening terminal and entering the command:
% 'ls /dev/tty.usb' and tab-completing.

fopen(M2USB);       % Open up the port to the M2 microcontroller.
flushinput(M2USB);  % Remove anything extranneous that may be in the buffer.

% Send initial packet to get first set of data from microcontroller
%fwrite(M2USB,1);% Send a packet to the M2.
time = 0;       % Set the start time to 0.
i = 1;          % Set i to 1, the indexer.
tic;            % Start timer.

%% Run program forever
try
while 1
    
    %% Read in data and send confirmation packet
    m2_buffer = fgetl(M2USB);   % Load buffer
    %fwrite(M2USB,1);            % Confirmation packet
    
    %% Parse microcontroller data
	% Expecting data in the form: [uint ADC1]
    %m2_ADC = hex2dec(m2_buffer(1:4));
    % Expecting data in the form: [int ANGLE]
%    keyboard

%For general use
    [target, remainder] = str2num(strtok(m2_buffer));
    [actual] = strtok(remainder);
    m2_buffer;
    time = toc; % Stamp the time the value was received
    
    % Remove the oldest entry.    
    Controller_target = circshift(Controller_target,-1);
    Controller_actual = circshift(Controller_actual,-1);
        
    %% Plotting
    figure(1);
    clf;
    hold on
    
    h(1)=plot(t, Controller_target(:,1),':or','LineWidth',2);
    h(2)=plot(t, Controller_actual(:,1),':og','LineWidth',2);
   
    legend(h,'target','actual');
    title('Controller');
    xlabel('Sample');
    ylabel('rps %');
    axis([0 maxPoints -1024 1024]);
    grid on
    pause(.04);
                                   
    hold off
    
    i=i+1;  % Incrememnt indexer
    %% Logging
%    if(rem(i,LOGFREQUENCY) == 0)
%        IMU_Log = [IMU_Log; time Angle_Actual];
%    end
    
end

catch
    %Close serial object
    fclose(M2USB);
end
