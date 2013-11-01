clear all
clc
 
answer=1; % this is where we'll store the user's answer

arduino=serial('/dev/ttyACM0','BaudRate',9600); % create serial communication object on port ttyACM0
 
fopen(arduino); % initiate arduino communication
 fprintf(arduino, '%c',char(uint16('Command1')));
fclose(arduino); % end communication with arduino
 return;
 
% while answer
%    answer=input('Enter the command you want to send: ');
%    myans = uint16(answer);
%    fprintf(arduino, '%c', char(myans));
% end
 
