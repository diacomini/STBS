%% Clear all
clear all;
close all;
clc;

%% Establish serial connection and establish general variables
% Sample code to open the serial port and set the baudrate came from here
% and from some of the links on MATLABs site
% https://www.mathworks.com/help/matlab/ref/serial.html

delete(instrfindall);   %pre-emptively close all ports
ports_list = seriallist;
%idx = strfind(ports_lsit, 'cu.usb');
%port = cell2mat(idx);
s1 = serial('/dev/cu.usbmodem14201', 'BaudRate', 19200);% Verify COM port

fopen(s1);
pause(2);
s1.ReadAsyncMode = 'continuous';
readasync(s1);
incoming_data = zeros(500,3); %establish a variable to store all of the data
time = zeros(500,1);
tension = zeros(500,1);
model_400 = zeros(500,1);
i = 1;%establish a counter to link the data points

%% Read and Store Orientation data
orientation_flag = 0;
while orientation_flag == 0
    fprintf(s1,'%s', '1');
    out = fscanf(s1);
    disp(out);
    
    if s1.BytesAvailable > 0
        orientation_flag = 1;
        sSerialData = fscanf(s1); %read sensor
        flushinput(s1);
        t = strsplit(sSerialData,'\t'); % same character as the Arduino code
        disp(t);
        roll = t(2);
        pitch = t(3);
        yaw = t(4);
    end
end
    

%% Acquire and real-time plot the data
% A lot of the code was prompted by watching this video and writing similar code from the source listed below.
% SPecifically the annimatedline function to plot real-time data.
% https://www.mathworks.com/videos/plotting-live-data-of-a-temperature-sensor-using-arduino-and-matlab-121317.html
figure
h = animatedline(0,0,'Color','k','LineWidth',1);
h1 = animatedline(0,0,'Color','b','LineWidth',1);
h2 = animatedline(0,0,'Color','r','LineWidth',1.5, 'LineStyle', '--');

% The idea to use a logarthmic Y axis was taken from 
% https://www.mathworks.com/matlabcentral/answers/279106-how-to-set-y-axis-as-log-scale
set(gca, 'YScale','log');
legend('Tension Sensor (kg)', 'Model 400', 'Location', 'West')
xlabel('Time (msec)')
% ax = gca;
% ax.YGrid = 'on';

%startTime = datetime('now');
for(l=1:500)  %wait until Arduino outputs data 
        sSerialData = fscanf(s1); %read sensor
        flushinput(s1);
        t = strsplit(sSerialData,'\t') % same character as the Arduino code
        incoming_data(i,1) = str2double(t(1)); %change index to correct / time
        incoming_data(i,2) = str2double(t(2)); %change index to correct / tension
        incoming_data(i,3) = str2double(t(3)); %change index to correct / model400

        time(i,1) = str2double(t(1)); %change index to correct / time
        tension(i,1) = str2double(t(2)); %change index to correct / tension
        model_400(i,1) = str2double(t(3)); %change index to correct / model400
        
        addpoints(h,incoming_data(i,1),incoming_data(i,2))% Add points to tension line
        addpoints(h1,incoming_data(i,1),incoming_data(i,3))% Add points to model400 line
        addpoints(h2,incoming_data(i,1),2.2)% Add points to model400 line
        drawnow
        i = i+1; %increase counter
end


%% Compile all of the orientation data into one variable
orientation = table(roll, pitch, yaw);
   

%% Save all data to excel file
  T = table(time, tension, model_400);
  filename = 'Test_A.xlsx';%create new file name for every test
% % Write table to file 
  writetable(orientation,filename,'Sheet',1)
  writetable(T,filename,'Sheet',2)

