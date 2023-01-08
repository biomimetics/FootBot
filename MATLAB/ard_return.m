function [force, raw] = ard_return(device)
%% the function reads message from arduino device
% and translate to force_reading and motor_step_status

message = readline(device);
message = convertStringsToChars(message);
message(end) = '';
raw = message;
if contains(message,'not')
    force = 0;
else
    var = sscanf(message,'%f %*s');
    force = var;
end

flush(device);

end
