function [message] = ard_command(device,mt_axis,mt_steps,mt_dir,scale)
%% the function translate arduino task into a string message need to print to the serial port
% it takes in device, motor_number,motor_steps,motor_direction(1 for up, -1
% for down),scale_read_or_not (1 for yes, 0 for no)
switch mt_axis
    case 'x'
        mt_num = 1;
    case 'y'
        mt_num = 2;
    case 'z'
        mt_num = 3;
end


if mt_steps < 0
    mt_steps = round(abs(mt_steps));
    mt_dir = -mt_dir;
end


if mt_dir <= 0
    mt_dir = 0;
elseif mt_dir >0
    mt_dir = 1;
end


if scale ~= 0
    scale = 1;
end


if mt_steps > 999999
    message = '>';
else
    message = sprintf('M_%d_%06d_%d_L_%d>',[mt_num, mt_steps, mt_dir, scale]);
end
flush(device);
writeline(device, message);

end
