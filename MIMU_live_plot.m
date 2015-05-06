%% ADC value liveplotting & logging.
%
% Reads ADC values from the M2 microcontroller.
% Plots the data realtime in a subplot.
% Logged into ADC_Log[time, ADC_Value] variable.
% Hit Ctrl-C to quit the program
%
% By Nick McGill
% Reads m_imu values from the M2 microcontroller and plots the desired data
% in real time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize program and USB port
% Close any existing open port connections
% For the first time running, comment this.
% if(exist('M2USB'))
%     fclose(M2USB);
% else
% fclose(instrfindall);
% end
%
%

%% If the above initialization does not work, please run the following commands manually and disconnect and reconnect USB.
% fclose(serial('COM5','Baudrate', 9600));
% fclose(instrfindall);
clear all;
close all;


%% VARIABLES
maxPoints = 20;         % Max number of data points displayed at a time.
t = 1:1:maxPoints;      % Create an evenly spaced matrix for X-axis.
IMU_Log = zeros(9,2);   % IMU data points logged here in format: [time ADC_Value]
IMU_live_plot_log = zeros(maxPoints,1); % Create an array to store ADC values.
IMU_ax = zeros(1,maxPoints);
IMU_ay = zeros(1,maxPoints);
IMU_az = zeros(1,maxPoints);
IMU_gx = zeros(1,maxPoints);
IMU_gy = zeros(1,maxPoints);
IMU_gz = zeros(1,maxPoints);
IMU_mx = zeros(1,maxPoints);
IMU_my = zeros(1,maxPoints);
IMU_mz = zeros(1,maxPoints);
%IMU_RANGE = -2000:2000;   % signed integer 16-bit



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
fwrite(M2USB,1);% Send a packet to the M2.
time = 0;       % Set the start time to 0.
i = 1;          % Set i to 1, the indexer.
tic;            % Start timer.

%% Run program forever
try
    while 1
        
        %% Read in data and send confirmation packet
        m2_buffer = fgetl(M2USB);   % Load buffer
        fwrite(M2USB,1);            % Confirmation packet
        
        %% Parse microcontroller data
        % Expecting data in the form: [uint ADC1]
        %m2_ADC = hex2dec(m2_buffer(1:4));
        % Expecting data in the form: [int ANGLE]
        %    keyboard
        %For use specifically with Acrobatv1.0 code
        % [RealinputACC, remainder] = strtok(m2_buffer);
        % RealinputACC = str2num(RealinputACC);
        % [RealinputGyr, Angle_Actual] =strtok(remainder);
        % RealinputGyr = str2num(RealinputGyr);
        % Angle_Actual = str2num(Angle_Actual);
        %For general use
        [m2_ax, remain] = strtok(m2_buffer);
        [m2_ay, remain2] = strtok(remain);
        [m2_az, remain3] = strtok(remain2);
        [m2_gx, remain4] = strtok(remain3);
        [m2_gy, remain5] = strtok(remain4);
        [m2_gz, remain6] = strtok(remain5);
        [m2_mx, remain7] = strtok(remain6);
        [m2_mz, remain8] = strtok(remain7);
        [m2_my] = strtok(remain8);
        m2_buffer;
        time = toc; % Stamp the time the value was received
        
        % Remove the oldest entry.
        IMU_ax = [str2double(m2_ax) IMU_ax(1:maxPoints-1)] ;
        IMU_ay = [str2double(m2_ay) IMU_ay(1:maxPoints-1)] ;
        IMU_az = [str2double(m2_az) IMU_az(1:maxPoints-1)] ;
        IMU_gx = [str2double(m2_gx) IMU_gx(1:maxPoints-1)] ;
        IMU_gy = [str2double(m2_gy) IMU_gy(1:maxPoints-1)] ;
        IMU_gz = [str2double(m2_gz) IMU_gz(1:maxPoints-1)] ;
        IMU_mx = [str2double(m2_mx) IMU_mx(1:maxPoints-1)] ;
        IMU_my = [str2double(m2_my) IMU_my(1:maxPoints-1)] ;
        IMU_mz = [str2double(m2_mz) IMU_mz(1:maxPoints-1)] ;
        
        %% Plotting
        figure(1);
        clf;
        hold on
        
        title('IMU values');
        
        
        subplot(3,1,1);
        hold on
        xlabel('Time');
        ylabel('accelerations');        
        axis([0 maxPoints -(2^14) 2^14]);
        plot(t, IMU_ax,':or','LineWidth',2);
        plot(t, IMU_ay,':og','LineWidth',2);
        plot(t, IMU_az,':om','LineWidth',2);
        
        subplot(3,1,2);
        hold on
        xlabel('Time');
        ylabel('gyros');  
        axis([0 maxPoints -(2^15) 2^15]);
        plot(t, IMU_gx,'-*r','LineWidth',2);
        plot(t, IMU_gy,'-*g','LineWidth',2);
        plot(t, IMU_gz,'-*m','LineWidth',2);

        subplot(3,1,3);
        hold on
        xlabel('Time');
        ylabel('magnetometer');
        axis([0 maxPoints -(2^10) 2^10]);
        plot(t, IMU_mx,'--r','LineWidth',2);
        plot(t, IMU_my,'--g','LineWidth',2);
        plot(t, IMU_mz,'--m','LineWidth',2);
        


        grid on
%         pause(.0001);
        
        hold off
        
        i=i+1;  % Incrememnt indexer
        %% Logging
        %    if(rem(i,LOGFREQUENCY) == 0)
        %        TODO
        %        IMU_Log = [IMU_Log; time Angle_Actual];
        %    end
        
    end
catch ME
    ME.stack
    %Close serial object
    fclose(M2USB);
end
