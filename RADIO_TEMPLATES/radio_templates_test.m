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

% paths
addpath('~/paris-osf/'); globalinit('silent');
addpath('~/Duke/Matlab/Library/system'); 
recaddpath('~/Duke/Matlab');
recaddpath('~/ReynoldsLab/Git-Arduino');
return

% open device connection
ard = serial('/dev/ttyACM0', 'Baud',19200, 'Terminator','LF', 'InputBuffersize',2^18, 'OutputBuffersize',2^12);
fopen(ard);

% close device connection
fclose(ard);




% *******************************************************************************************************
% CHECK AD41020 CONTROL / PLL FREQUENCIES

% Arduino
%     basic setup
fprintf(ard, 'TIME:DWELL 15383');
fprintf(ard, 'TIME:PULSE  7500');
fprintf(ard, 'TIME:PAUSE     0');
%     "run" commands
fprintf(ard, 'RUN:FREQ');
fprintf(ard, 'CONT:FREQ');
fprintf(ard, 'STOP');
%     display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end

% expected PLL frequencies
fprintf(ard, 'SWEEP:DEFAULT');
f_pll = 10415e6 : 45e6 : 13070e6;

% EXA signal analyzer (semi-automatic)
exa = exa_n9010a_ctrl('192.168.1.97', 'open', 1e6);
exa_n9010a_ctrl(exa, 'set-ampl-reflevel', 10);
exa_n9010a_ctrl(exa, 'set-freq-start-stop', [f_pll(1) - 5*diff(f_pll(1:2)), f_pll(end) + 5*diff(f_pll(1:2))]);
exa_n9010a_ctrl(exa, 'set-hold', 'clear');
exa_n9010a_ctrl(exa, 'set-numpoints', 20001); % highest res possible
%     frequency vector
f_meas = exa_n9010a_ctrl(exa, 'get-freq'); 
f_meas = linspace(f_meas(1), f_meas(2), exa_n9010a_ctrl(exa, 'get-numpts'));
f_res  = diff(f_meas(1:2));

% set to continuous run and max-hold
fprintf(ard, 'CONT:FREQ'); pause(0.1);
exa_n9010a_ctrl(exa, 'set-hold', 'clear');
exa_n9010a_ctrl(exa, 'set-hold', 'max');
%     ... wait until the spectrum is fully filled, then transfer data
pause(25); 
psd = exa_n9010a_ctrl(exa, 'get-trace-data', 1);
%     extract peak power level
[pwr, ind] = findpeaks(psd, 'minpeakheight',-10, 'npeaks',numel(f_pll), 'minpeakdistance', round(mean(diff(f_pll))/mean(diff(f_meas)) * 0.8));

% plots
%     frequency offset
close all; figure; hold on;
plot(f_pll/1e9, (f_pll - f_meas(ind))/1e3);
plot(f_pll/1e9, -ones(size(f_pll)) * f_res/1e3, 'r--');
plot(f_pll/1e9,  ones(size(f_pll)) * f_res/1e3, 'r--');
plot(f_pll/1e9, zeros(size(f_pll)), 'k-');
hold off; grid on; xlim(xyzlimits(f_pll)/1e9); ylim(xyzlimits((f_pll-f_meas(ind)), f_res, -f_res)/1e3);
setlabels('ACCURACY OF SET SYNTHESIZER FREQUENCY (ADF41020, Arduino controlled)', 'f_{set,PLL} [GHz]', 'f_{set,PLL} - f_{meas} [kHz]');
setlegend({'measurement', '+/- FFT resolution'}, 'NorthEast');

%     power levels
close all; figure; plot(f_meas(ind)/1e9, pwr);
hold off; grid on; xlim(xyzlimits(f_meas(ind))/1e9); ylim(xyzlimits(pwr));
setlabels('SYNTHESIZER OUTPUT POWER (ADF41020, Arduino controlled)', 'f [GHz]', 'power [dBm]');




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
   if ~isempty(strfind(reply, err_msg1))
      fprintf('Test %3i, len=%4i:   OK (unknown command message)\n', i, length(str));
   elseif ~isempty(strfind(reply, err_msg2))
      fprintf('Test %3i, len=%4i:   OK (input buffer overrun message)\n', i, length(str));
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
   'PLL:PFD %i', 'PLL:PFD?', 'TIME:DWELL 500', 'TIME:DWELL?', 'TIME:PAUSE %i', 'TIME:PAUSE?', 'TIME:PULSE %i', 'TIME:PULSE?',...
   'SWEEP:POINTS?', 'SWEEP:DEFAULT', 'SWEEP:LIN %i,%i,%i', 'SWEEP:LIST %i,%i,%i,%i,%i,%i,%i,%i,%i,%i', ...
   'SWITCH:NUM %i', 'SWITCH:NUM?', 'SWITCH:PORTS %i', 'SWITCH:PORTS?', 'SWITCH:STATES %i,%i,%i', 'SWITCH:STATES?', 'SWITCH:SELECT %i', };
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
fmin  =  7000; % MHz
fmax  = 14000; % MHz
maxpts =  101; % maximum number of points
tpause = [0.3, 3, 0.2]; % s pause time [linear, list, #pts]

% program a few sweeps and test the results
clc;
for i = 1 : 25
   
   % choose random channel spacing
   fchan = randi([1, 5]); % MHz
   %     set PFD frequency
   fprintf(ard, sprintf('PLL:PFD %i', fchan/4 * 1000)); % kHz
   pause(tpause(3));
   flushinput(ard);
   %     check new channel spacing
   if query(ard, 'PLL:PFD?\n', '%s','%i') ~= fchan/4*1000
      error('--- STOP: PFD ---');
   end
   
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
   end
   % output
   fprintf('\nTEST %i:     %5i : %4i : %5i MHz   (%3i values)\n', i, fstart,fstep,fstop, length(f));
   
   % program this sweep as linear sweep
   cmd = sprintf('SWEEP:LIN %i,%i,%i',fstart,fstep,fstop);
   flushinput(ard);
   fprintf(ard, cmd);
   fprintf('      %s%s  |', cmd, repmat(' ',1,80-length(cmd)));
   pause(tpause(1));
   %     reply (plus get any debug information)
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   if ~strcmpi(reply{end}, ['OK -- ',regexprep(cmd,',',':')]); error('--- STOP: HANDSHAKE ---'); end
   fprintf(' %s%s |', reply{end}, repmat(' ',1,40-length(reply{end})));
   %     number of frequencies
   flushinput(ard);
   fprintf(ard, 'SWEEP:POINTS?');
   pause(tpause(3));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   fprintf(' #PTS: %s%s |', reply{end}, repmat(' ',1,10-length(reply{end})));
   if str2double(reply{end})<length(f); error('--- STOP: #pts ---'); end
   %     register values
   flushinput(ard);
   fprintf(ard, 'PLL:REGVALS?');
   pause(tpause(2));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   clear F R N reg_chk_ok;
   for j = 1 : length(f)
      [~, F,R,N] = adf41020_calc_regs(f(j), 100, fchan/4*1000); % expected register values
      Regs = sscanf(reply{j}, '%X|%X|%X'); % returned register values
      reg_chk_ok(j) = all( Regs(:) - double([F; R; N]) < eps ); % compare
   end
   if all(reg_chk_ok); fprintf(' PLL REGS: OK \n'); else error('--- STOP: registers ---'); end
 
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
   fprintf(' %s%s |', reply{end}, repmat(' ',1,40-length(reply{end})));
   %     number of frequencies
   flushinput(ard);
   fprintf(ard, 'SWEEP:POINTS?');
   pause(tpause(3));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   fprintf(' #PTS: %s%s |', reply{end}, repmat(' ',1,10-length(reply{end})));
   if str2double(reply{end})<length(f); error('--- STOP: #pts ---'); end
   %     register values
   flushinput(ard);
   fprintf(ard, 'PLL:REGVALS?');
   pause(tpause(2));
   reply={}; ri=1; while ard.BytesAvailable > 0; reply{ri}=strtrim(fgetl(ard)); ri=ri+1; end
   clear F R N reg_chk_ok;
   for j = 1 : length(f)
      [~, F,R,N] = adf41020_calc_regs(f(j), 100, fchan/4*1000); % expected register values
      Regs = sscanf(reply{j}, '%X|%X|%X'); % returned register values
      reg_chk_ok(j) = all( Regs(:) - double([F; R; N]) < eps ); % compare
   end
   if all(reg_chk_ok); fprintf(' PLL REGS: OK \n'); else error('--- STOP: registers ---'); end
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
fprintf(ard, 'TIME:PULSE?');
fprintf(ard, 'SWEEP:POINTS?');
fprintf(ard, 'SWITCH:NUM?');
fprintf(ard, 'SWITCH:PORTS?');
fprintf(ard, 'SWITCH:STATES?');

% PLL register values
fprintf(ard, 'SWEEP:DEFAULT'); pause(0.1);
fprintf(ard, 'PLL:REGVALS?');

% display output (IMPORTANT: do not execute at the same time as fprintf commands)
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end


% *********************************
% SET COMMANDS

fprintf(ard, sprintf('PLL:PFD %i', randi([1000, 5000])));
fprintf(ard, 'PLL:PFD?');

fprintf(ard, sprintf('TIME:DWELL %i', randi([500, 15000])));
fprintf(ard, 'TIME:DWELL?');

fprintf(ard, sprintf('TIME:PULSE %i', randi([500, 15000])));
fprintf(ard, 'TIME:PULSE?');

fprintf(ard, sprintf('TIME:PAUSE %i', randi([500, 15000])));
fprintf(ard, 'TIME:PAUSE?');

fprintf(ard, sprintf('SWITCH:NUM %i', randi([0, 12])));
fprintf(ard, 'SWITCH:NUM?');

fprintf(ard, sprintf('SWITCH:PORTS %i', randi([1, 8])));
fprintf(ard, 'SWITCH:PORTS?');

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
% SWITCHES

% maximum number of switches exeeded
fprintf(ard, 'SWITCH:NUM 15000');
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end

% random number of ports and switches
fprintf(ard, sprintf('SWITCH:NUM %i', randi([0, 12])));
fprintf(ard, sprintf('SWITCH:PORTS %i', randi([1, 8])));
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end
num_sw = query(ard, 'SWITCH:NUM?', '%s\n', '%i');
num_po = query(ard, 'SWITCH:PORTS?', '%s\n', '%i');

% random switch states
sw_states = randi([0, 2^num_po-1], [1,num_sw]);
fprintf(ard, sprintf('SWITCH:STATES %i,%i,%i', sw_states));
fprintf(ard, 'SWITCH:STATES?');
disp(sw_states);
while ard.BytesAvailable > 0; disp(strtrim(fgetl(ard))); end

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

