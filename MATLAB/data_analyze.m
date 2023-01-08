clear; close all; clc;

for in = 1:5
    filename{in}=sprintf("trial_%d.mat",in);
    load(filename{in});
    force_all{in} = force;
    ch0_all{in} = ch0;
    ch1_all{in} = ch1;
    ch2_all{in} = ch2;
    steps_all{in} = steps_status;
end

for i = 1:5
    figure(1);
    hold on;
    plot(force_all{i}, -steps_all{i}(1)+steps_all{i});
    figure(2);
    hold on;
    plot(force_all{i},ch0_all{i});
    figure(3);
    hold on;
    plot(force_all{i},ch1_all{i});
    figure(4);
    hold on;
    plot(force_all{i},ch2_all{i});
end

figure(1);
title("Force Vs. Motor Steps");
xlabel("Force (mN)");
ylabel("Motor Steps");
grid on;
legend("trial-1","trial-2","trial-3","trial-4","trial-5");

figure(5);
for i=1:5
    plot(force_all{i},ch0_all{i},'-r');
    hold on;
    plot(force_all{i},ch1_all{i},'-b');
    plot(force_all{i},ch2_all{i},'-g');
end
title("Force Vs. Capacitance Reading - Channel 1-3");
legend("Channel-0","Channel-1","Channel-2");
grid on;

for i=2:4
    figure(i);
    title(sprintf("Force Vs. Capacitance Reading - Channel %d",i-2));
    xlabel("Force(mN)");
    ylabel("Capacitance Reading");
    grid on;
    legend("trial-1","trial-2","trial-3","trial-4","trial-5");
end