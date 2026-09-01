clear ; close all; clc;
format compact;

%% Parameter Setup
% ard = serialport("/dev/cu.usbmodem1301",9600);
sensor = serialport("/dev/cu.usbmodem214201",9600);

pause(3);
flush(sensor);

tStart = tic; 
count = 0;

%% Start
for i = 1:600
    count = count+1;
    [ch0(count), ch1(count), ch2(count)] = sensor_return(sensor);
    T(count) = toc(tStart);
end

plot(T);
ylabel('time(s)');
xlabel('number of commands sent')