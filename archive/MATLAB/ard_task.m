function [command_raw,force_reading, reading_raw] = ard_task(device,mt_axis,mt_steps,mt_dir,scale)
%% The function Commands the Arduino and Read Data from Arduino
command_raw = ard_command(device,mt_axis,mt_steps,mt_dir,scale);
if scale ~= 1
    scale = 0;
end

pause(mt_steps/500*1.1 + scale*1);

[force_reading, reading_raw] = ard_return(device);
end
