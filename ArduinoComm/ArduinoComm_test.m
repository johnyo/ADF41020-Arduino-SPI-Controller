% *******************************************************************************************************
% Test Script for ArduinoComm.ino  (Serial Control Interface)
%
% IMPORTANT: Arduino software (in particular the serial monitor) should be closed while using this
%            script. Otherwise two devices access the same serial device, which will lead to confusion
%            and might cause errors.
% *******************************************************************************************************

% init
clear; close all; clc; pause(0.01);
return

% open device connection
ard = serial('/dev/ttyACM1', 'Baud',9600, 'Terminator','LF', 'InputBuffersize',2^18, 'OutputBuffersize',2^12);
fopen(ard);

% close device connection
fclose(ard);



% *******************************************************************************************************
% MANUAL SERIAL INTERFACE ROBUSTNESS TESTING: RANDOM STRINGS

% random commands of random length
%     settings
str_len = 700; % bytes maximum length (choose slightly larger than SERIAL_INBUF_SIZE)
char_set = [0, 255]; % all 1-byte characters (this might cause problems after some time)
char_set = [48, 122]; % 0-9, a-z, A-Z, and a few special characters
char_set = [32, 126]; % all printable ASCII
%     create command
len = randi([1, str_len], 1);
str = char(randi(char_set, [1,len]));
%     send
fprintf(ard, str);

% FREQ commands with variable length
%     settings
maxpts = 60; % maximum number of points
%     create command
test_vec = randi(16383, [1,randi(maxpts,1)]);
cmd = sprintf('%s %s', 'FREQ', sprintf('%i,', test_vec(:)));
%     send
fprintf(ard, cmd);


% display output for the above commands (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end


