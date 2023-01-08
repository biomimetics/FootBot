clear ; close all; clc;
format compact;

%% Parameter Setup
ard = serialport("/dev/cu.usbmodem21301",9600);
%sensor = serialport("COM4",9600);

pause(3);
flush(ard);
tStart = tic; 
count = 0;
%% Start
for i = 1:10
    count = count+1;
    message = ard_task(ard,1,i,-1,0);
    [force(count), steps_status(count), reading{count}] = ard_return(ard);
    sprintf(reading{count});
    T(count) = toc(tStart);
end

for i = 1:12
    count = count+1;
    message = ard_task(ard,1,i,-1,1);
    [force(count), steps_status(count), reading{count}] = ard_return(ard);
    sprintf(reading{count});
    T(count) = toc(tStart);
end

plot(T);
ylabel('time(s)');
xlabel('number of commands sent')
clear ard;