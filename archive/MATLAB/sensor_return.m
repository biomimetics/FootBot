function [ch0 ch1 ch2 temp] = ard_return(device)
%% the function reads message from arduino device
% and translate to force_reading and motor_step_status
text_length = 0;
flush(device);
pause(0.05);
while text_length <=10
    text = readline(device);
    text = convertStringsToChars(text);
    text_length = length(text);
end

text(end) = [];
netindex = 1;
var = sscanf(text ,'%f %*s %f %*s %f %*s %f %*s %f');
ch0 = var(2);
ch1 = var(3);
ch2 = var(4);
temp = var(5);

end



