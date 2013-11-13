% *******************************************************************************************************
% Test Script for RADIO_TEMPLATES.ino  (Serial Control Interface)
%   Author: Daniel Arnitz
%
% IMPORTANT: Arduino software (in particular the serial monitor) should be closed while using this
%            script. Otherwise two devices access the same serial device, which will lead to confusion
%            and might cause errors.
% *******************************************************************************************************

% init
clear; close all; clc; pause(0.01);
return

% open device connection
ard = serial('/dev/ttyACM1', 'Baud',19200, 'Terminator','LF', 'InputBuffersize',2^18, 'OutputBuffersize',2^12);
fopen(ard);

% close device connection
fclose(ard);



% *******************************************************************************************************
% SERIAL INTERFACE ROBUSTNESS TESTING: RANDOM STRINGS

% IMPORTANT: reset Arduino before starting this test!

% settings
str_len = 700; % bytes maximum length (choose slightly larger than SERIAL_INBUF_SIZE)
char_set = [0, 255]; % all 1-byte characters (this might cause problems after some time)
char_set = [48, 122]; % 0-9, a-z, A-Z, and a few special characters
char_set = [32, 126]; % all printable ASCII
err_msg1 = 'ERROR -- Unrecognized command';
err_msg2 = 'ERROR -- Input buffer overrun. Resetting.';

% run random commands, check if Arduino reacts with error message
for i = 1 : 25
   % reset
   reply = '';
   flushinput(ard);
   flushoutput(ard);
   % create random string with random length
   len = randi([1, str_len], 1);
   str = char(randi(char_set, [1,len]));
   % send this command
   fprintf(ard, str);
   % give the Arduino time to process this
   pause(0.5);
   % get reply from buffer
   if ard.BytesAvailable > 0
      reply = strtrim(fgetl(ard));
   end
   % check
   if ~isempty(strfind(reply, err_msg1)) || ~isempty(strfind(reply, err_msg2))
      fprintf('Test %3i, len=%4i:   OK (received error message)\n', i, length(str));
   else
      fprintf('Test %3i, len=%4i:   WARNING (received truncated or corrupted error message)\n', i, length(str));
      str
      reply
   end
end

% check if commands still work
flushinput(ard);
flushoutput(ard);
fprintf(ard, 'STOP');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end



% *******************************************************************************************************
% SERIAL INTERFACE ROBUSTNESS TESTING: RANDOM COMMANDS

% IMPORTANT: reset Arduino before starting this test!

% command list
cmd_list = {'RUN:PANELS', 'RUN:FREQ', 'CONT:PANELS', 'CONT:FREQ', 'STOP', 'SINGLE',...
   'PLL:PFD %i', 'PLL:PFD?', 'TIME:DWELL 500', 'TIME:DWELL?', 'TIME:PAUSE %i', 'TIME:PAUSE?', 'SWEEP:POINTS?'...
   'SWEEP:DEFAULT', 'SWEEP:LIN %i,%i,%i', 'SWEEP:LIST %i,%i,%i,%i,%i,%i,%i,%i,%i,%i', ...
   'SWITCH:STATES %i,%i,%i', 'SWITCH:STATES?', 'SWITCH:SELECT %i', };
% send commands in random order, output Arduino's reply
for i = 1 : 10
   % reset
   reply = '';
   flushinput(ard);
   flushoutput(ard);
   % choose random command
   ind = randi([1, length(cmd_list)], 1);
   cmd = cmd_list{ind};
   % replace %i with random values (not too large; otherwise a started sweep might take ages)
   ind_val = strfind(cmd, '%i');
   for j = fliplr(ind_val)
      cmd = sprintf('%s%i%s', cmd(1:j-1), randi([1,1000],1), cmd(j+2:end));
   end
   % print command
   fprintf('%s%s  |', cmd, repmat(' ',1, 80-length(cmd)));
   % send this command
   fprintf(ard, cmd);
   % give the Arduino time to process this
   pause(1);
   % get reply from buffer
   if ard.BytesAvailable > 0
      reply = strtrim(fgetl(ard));
   end
   % output reply
   fprintf('  %s\n', reply);
end

% check if commands still work
flushinput(ard);
flushoutput(ard);
fprintf(ard, 'STOP');
pause(1); while ard.BytesAvailable > 0; disp(fgetl(ard)); end



% *******************************************************************************************************
% SWEEP SETUP COMMANDS
%     [TODO] add register value checks

% IMPORTANT: reset Arduino before starting this test!

% reset interface
flushinput(ard);
flushoutput(ard);

% linear sweep (PFD has to be correct => load from Arduino)
fmin  =  7000;
fchan = 4 * query(ard, 'PLL:PFD?\n', '%s','%i') / 1e3; % has to be correct; otherwise the PLL registers are crap
fmax  = 14000;
maxpts = 101; % maximum number of points
tpause = [0.3, 3, 0.2]; % s pause time [linear, list, #pts]

% program a few sweeps and test the results
clc;
for i = 1 : 25
   % random start, step, stop (multiples of fchan; but we don't care if fstart and fstop are multiples of fstep apart)
   fstart = round((fmin + rand * (fmax-fmin)) / fchan) * fchan;
   fstop  = round((fmax - rand * (fmax-fmin)) / fchan) * fchan;
   if fstart > fstop; tmp = fstart; fstart = fstop; fstop = tmp; end
   fstep = (1 + round((fstop-fstart)/randi([1,maxpts-2]) / fchan)) * fchan;
   % calculate frequency vector
   f = fstart : fstep : fstop;
   % quick checks
   if round(fstep/fchan) ~= fstep/fchan || length(f) > maxpts
      error('Problems with frequency vector generation.');
      continue;
   end
   % output
   fprintf('\n%3i  -  %5i : %4i : %5i MHz   (%3i values)\n', i, fstart,fstep,fstop, length(f));
   
   % program this sweep as linear sweep
   cmd = sprintf('SWEEP:LIN %i,%i,%i',fstart,fstep,fstop);
   flushinput(ard);
   fprintf(ard, cmd);
   fprintf('      %s%s  |', cmd, repmat(' ',1,80-length(cmd)));
   pause(tpause(1));
   %     reply (plus get any debug information)
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   if ~strcmpi(reply{end}, ['OK -- ',regexprep(cmd,',',':')]); error('--- STOP: HANDSHAKE ---'); end
   fprintf(' %s%s |', reply{end}, repmat(' ',1,60-length(reply{end})));
   %     number of frequencies
   flushinput(ard);
   fprintf(ard, 'SWEEP:POINTS?');
   pause(tpause(3));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   fprintf(' #PTS: %s%s\n', reply{end}, repmat(' ',1,20-length(reply{end})));
   if str2double(reply{end})<length(f); error('--- STOP: #pts ---'); end
   
   % program this sweep as randomized list sweep
   f = f(randperm(length(f)));
   cmd = sprintf('SWEEP:LIST %s', sprintf('%i,', f)); cmd(end) = [];
   flushinput(ard);
   fprintf(ard, cmd);
   fprintf('      %s%s  |', cmd(1:min(80,length(cmd))), repmat(' ',1,80-min(80,length(cmd))));
   pause(tpause(2));
   %     reply (plus get any debug information)
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   if ~strcmpi(reply{end}, 'OK -- SWEEP:LIST'); error('--- STOP: HANDSHAKE ---'); end
   fprintf(' %s%s |', reply{end}, repmat(' ',1,60-length(reply{end})));
   %     number of frequencies
   flushinput(ard);
   fprintf(ard, 'SWEEP:POINTS?');
   pause(tpause(3));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   fprintf(' #PTS: %s%s\n', reply{end}, repmat(' ',1,20-length(reply{end})));
   if str2double(reply{end})<length(f); error('--- STOP: #pts ---'); end
end



% *********************************
% LONG COMMAND (MANUAL TESTING)

% open device connection
ard = serial('/dev/ttyACM1', 'Baud',19200, 'Terminator','LF', 'InputBuffersize',2^18, 'OutputBuffersize',2^12);
fopen(ard);
fclose(ard);

cmd = 'SWEEP:LIST 12425,10265,11645,11885,12905,10625,11105,9965';
fprintf(ard, cmd);

cmd = 'SWEEP:LIST 12425,10265,11645,11885,12905,10625,11105,9965,9005,11285,9425,9785,9065,12365,10145,9245,12125,11525,9125,12545,12245,12305,10325,10445,11345,12185,10565,8285,11165,8345,11825,11765,11945,11705,12485,8465,10025,9845,11045,12665,10385,10085,8705,9905,9725,8525,9665,8825,12845,10505,12005,10205,8765,10745,10805,10865,8585,9365,10925,9185,11465,11225,10985,9485,8405,8945,12605,9545,10685,12725,8885,8645,9605,11405,12785,9305,12065,11585';
fprintf(ard, cmd);

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end





% *******************************************************************************************************
% MANUALLY TEST COMMANDS

% *********************************
% "RUN" COMMANDS

fprintf(ard, 'RUN:PANELS');
fprintf(ard, 'RUN:FREQ');
fprintf(ard, 'CONT:PANELS');
fprintf(ard, 'CONT:FREQ');
fprintf(ard, 'STOP');
fprintf(ard, 'SINGLE');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end


% *********************************
% "QUERY" COMMANDS

fprintf(ard, 'PLL:PFD?');
fprintf(ard, 'TIME:DWELL?');
fprintf(ard, 'TIME:PAUSE?');
fprintf(ard, 'SWEEP:POINTS?');
fprintf(ard, 'SWITCH:STATES?');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end

% PLL register values
fprintf(ard, 'SWEEP:DEFAULT'); pause(0.1);
fprintf(ard, 'PLL:REGVALS?');


% *********************************
% SET COMMANDS

fprintf(ard, sprintf('PLL:PFD %i', randi([1000, 5000])));
fprintf(ard, 'PLL:PFD?');

fprintf(ard, sprintf('TIME:DWELL %i', randi([500, 15000])));
fprintf(ard, 'TIME:DWELL?');

fprintf(ard, sprintf('TIME:PAUSE %i', randi([500, 15000])));
fprintf(ard, 'TIME:PAUSE?');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end


% *********************************
% CHECK IF CHANGING SWEEP CORE PARAMETERS RESETS THE SWEEP

% PFD frequency
fprintf(ard, 'SWEEP:DEFAULT');
fprintf(ard, 'SWEEP:POINTS?');
fprintf(ard, sprintf('PLL:PFD %i', randi([1000, 5000])));
fprintf(ard, 'SWEEP:POINTS?');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end


% *********************************
% CHECK IF CHANGING SWEEP CORE PARAMETERS RESETS THE SWEEP

% random switch states
sw_states = randi([0, 255], [1,3]);
fprintf(ard, sprintf('SWITCH:STATES %i,%i,%i', sw_states));
fprintf(ard, 'SWITCH:STATES?');
disp(sw_states);

% running light
for i = 1 : 1 : 3 * 6
   fprintf(ard, sprintf('SWITCH:SELECT %i', i));
   fprintf(ard, 'SWITCH:STATES?');
   pause(0.1);
end

% reset
fprintf(ard, 'SWITCH:STATES 0,0,0');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end

