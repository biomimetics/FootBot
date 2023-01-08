close all; clc;
format compact;
tStart = tic;

%% Device Setup
ard = serialport("/dev/cu.usbmodem214101",9600);          % Open Arduino Serial Port
sensor = serialport("/dev/cu.usbmodem21301",9600);        % Open Sensor Serial Port 

pause(5);          % Wait until MATLAB Set connections with Serial ports
flush(sensor);     % Flush out all serial port readings from sensor
flush(ard);        % Flush out all serial port readings from arduino
x = 'x';
y = 'y';
z = 'z';

%% Parameters Setup
mm_per_round = [6.35, 6.35, 1.27];  % XYZ s   crew's length per full round
no_per_round = [200 200 200];       % XYZ Screw's number of turn per full round
mm_per_step = mm_per_round./no_per_round;

theta =[0:30:360];                % Defines Polar Coordinates (Angle) of the force location
theta(end) = [];                   
radius = [0:5:25];               % Defines Polar Coordinates (Radius) of the force location

force_min = 5;                    % Minimum force to determine the initial of touch
number_each_loc = 4000;              % Number of Steps Down for each location with data collection
total_number_points = number_each_loc * length(theta) * length(radius);
ch0 = zeros(1,total_number_points);
ch1 = zeros(1,total_number_points);
ch2 = zeros(1,total_number_points);
force = zeros(1,total_number_points); 
time = zeros(1,total_number_points);
message = cell(1,total_number_points);
reading = cell(1,total_number_points);
polar_radius = zeros(1,total_number_points);
polar_theta = zeros(1,total_number_points);


%% System Initialize
% Step 1: Adjust XYZ motor to align the tip of force applier to the center surface of the testing object
% Step 2: Rotate the Z axis motor up for 3 cm for safety

ard_task(ard,z,200,1,0);             % Motor Z drive up 200 steps for safety

sprintf("System Initializing \n");
ard_task(ard,z,180,-1,0);            % Motor Z drive back down 180 steps for safety
steps_down_raw = 20;
force_temp = 0;

while force_temp <= force_min
    [message_temp force_temp reading_temp]= ard_task(ard,z,steps_down_raw,-1,1);
end

sprintf("Touched \n")
sprintf("Motor Going Back up")

temp = ard_task(ard,z,steps_down_raw,1,0);

sprintf("System Ready \n");

flush(sensor);                     % Flush out all serial port readings from sensor
flush(ard);                        % Flush out all serial port readings from arduino



%% Start Testing
count = 0;
safety_lift = 800;
% Lift extra Space (Z-axis) for avoid contact before move x-y
ard_task(ard,z,safety_lift,1,0);
radius_avoid = [20];
theta_avoid = [60, 180, 300];
zero_zero = 0;
surface_dis = 200;

for i=1:length(theta)
    for k=1:length(radius)

        % Motor move to correct x y coordinates
        x_move = round(radius(k) * cosd(theta(i)) / mm_per_step(1));
        y_move = round(radius(k) * sind(theta(i)) / mm_per_step(2));
        ard_task(ard,x,x_move,1,0);
        ard_task(ard,y,y_move,1,0);
        
        % Motor move down in z-axis to the start point before safety-lift
        ard_task(ard,z,safety_lift,-1,0);
        
        flush(sensor);          % Flush out all serial port readings from sensor
        flush(ard);             % Flush out all serial port readings from arduino

        % Testing Starts and Data Collecting
        
        for index = 1:(number_each_loc+surface_dis)

            if  (radius(k)==0) && (zero_zero ~= 0)
                break;
            end

            if ismember(radius(k),radius_avoid) && ismember(theta(i),theta_avoid)
                break;
            end

            count = count + 1;
            [message{count}, force(count), reading{count}] = ard_task(ard,z,1,-1,1);
            [ch0(count), ch1(count), ch2(count)] = sensor_return(sensor);
            time(count) = toc(tStart);
            polar_theta(count) = theta(i);
            polar_radius(count) = radius(k);
            if force(count)>=2500
                break;
            end
        end

        if (radius(k)==0)
            zero_zero = 1;
        end

        ard_task(ard,z,index+safety_lift+surface_dis,1,0);        % Return Z back above Safety-lift
        pause(10);
        ard_task(ard,x,x_move,-1,0);                              % Return to X = 0;
        ard_task(ard,y,y_move,-1,0);                              % Return to Y = 0;
    end
end
ard_task(ard,z,200,1,0);
%figure(1);
%plot(time(1:count));
%ylabel('time(s)');
%xlabel('number of commands sent')

str=sprintf('all_trial_%d',no_trial);
save(str,'force','ch0','ch1','ch2','time','polar_radius','polar_theta','message','reading');
