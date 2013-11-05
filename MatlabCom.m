clear all
clc
 
answer=1; % this is where we'll store the user's answer
arduino=serial('/dev/ttyACM0','BaudRate',9600); % create serial communication object on port COM4
 
% fopen(arduino); % initiate arduino communication
%  fprintf(arduino, '%c',char(uint16('buzz')));
% fclose(arduino); % end communication with arduino
%  return;

while answer
answer=input('Enter the command you want to send: ');
fopen(arduino); % initiate arduino communication
 fprintf(arduino, '%c',char(uint16(answer)));
fclose(arduino); % end communication with arduino
end
 
fclose(arduino); % end communication with arduino
