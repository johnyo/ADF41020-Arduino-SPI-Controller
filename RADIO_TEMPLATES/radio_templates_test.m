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
ard = serial('/dev/ttyACM0', 'Baud',9600, 'Terminator','LF', 'InputBuffersize',2048, 'OutputBuffersize',2048);
fopen(ard);

% close device connection
fclose(ard);



% *******************************************************************************************************
% SERIAL INTERFACE ROBUSTNESS TESTING: RANDOM STRINGS

% IMPORTANT: reset Arduino before starting this test!

% settings
str_len = 1200; % bytes maximum length (choose slightly larger than SERIAL_INBUF_SIZE)
char_set = [0, 255]; % all 1-byte characters (this might cause problems after some time)
char_set = [48, 122]; % 0-9, a-z, A-Z, and a few special characters
char_set = [32, 126]; % all printable ASCII
err_msg = 'ERROR -- Unrecognized command';

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
   if ~isempty(strfind(reply, err_msg))
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
   'PLL:PFD %i', 'PLL:PFD?', 'TIME:DWELL 500', 'TIME:DWELL?',  'TIME:PAUSE %i', 'TIME:PAUSE?', 'SWEEP:POINTS?'...
   'SWEEP:DEFAULT', 'SWEEP:LIN %i,%i,%i', 'SWEEP:LIST %i,%i,%i,%i,%i,%i,%i,%i,%i,%i', ...
   'SWITCH:SELECT %i', };
% send commands in random order, output Arduino's reply
for i = 1 : 25
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
   pause(0.5);
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
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end



% *******************************************************************************************************
% SWEEP SETUP COMMANDS
%     [TODO] add register value checks

% IMPORTANT: reset Arduino before starting this test!

% reset interface
flushinput(ard);
flushoutput(ard);

% linear sweep (PFD has to be correct => load from Arduino)
fmin  = 10000;
fchan = 4 * query(ard, 'PLL:PFD?\n', '%s','%i') / 1e3; % has to be correct; otherwise the PLL registers are crap
fmax  = 15000;
maxpts = 91; % maximum number of points
tpause = 0.5; % s pause time

% several runs
clc
for i = 1 : 10
   % reset
   reply = '';
   % random start, step, stop (multiples of fchan; but we don't care if fstart and fstop are multiples of fstep apart)
   fstart = round((fmin + rand * (fmax-fmin)) / fchan) * fchan;
   fstep  = round( max(fchan, min((fmax-fmin)/(maxpts-1), rand*(fmax-fmin)/2)) /fchan) * fchan;
   fstop  = round((fmax - rand * (fmax-fmin)) / fchan) * fchan;
   if fstart > fstop
      tmp = fstart; fstart = fstop; fstop = tmp;
   end
   % calculate frequency vector
   f = fstart : fstep : fstop;
   % quick checks
   if round(fstep/fchan) ~= fstep/fchan || length(f) > maxpts
      disp('PROBLEMS...');
      continue;
   end
   % output
   fprintf('\n%3i  -  %5i : %4i : %5i MHz   (%3i values)\n', i, fstart,fstep,fstop, length(f));
   
   % program this sweep as linear sweep 
   cmd = sprintf('      SWEEP:LIN %i,%i,%i',fstart,fstep,fstop);
   fprintf(ard, cmd);
   fprintf('%s%s  |', cmd, repmat(' ',1,80-length(cmd)));
   pause(tpause); 
   %     get reply from buffer and check
   if ard.BytesAvailable > 0
      reply = strtrim(fgetl(ard));
   end
   if ~isempty( strfind(reply, sprintf('OK -- SWEEP:LIN %i:%i:%i',fstart,fstep,fstop)) )
      fprintf('  reply: OK    |');
   else
      fprintf('  reply: ERROR |');
      disp(reply);
   end
   %     check number of frequencies
   flushinput(ard);
   npts = query(ard, 'SWEEP:POINTS?\n', '%s','%i');
   if npts == length(f)
      fprintf('  #pts: OK    \n');
   else
      fprintf('  #pts: ERROR  (has %i, should be %i) \n', npts, length(f));
   end
   
   % program this sweep as list sweep 
   cmd = sprintf('      SWEEP:LIST %s', sprintf('%i,', f)); cmd(end) = [];
   fprintf(ard, cmd);
   if length(cmd) <= 80
      fprintf('%s%s  |', cmd, repmat(' ',1,80-length(cmd)));
   else
      fprintf('%s...%s  |', cmd(1:min(77, length(cmd))), repmat(' ',1,77-min(77,length(cmd))));
   end
   %     give the Arduino time to process this
   pause(tpause);
   %     get reply from buffer and check
   if ard.BytesAvailable > 0
      reply = strtrim(fgetl(ard));
   end
   if ~isempty( strfind(reply, 'OK -- SWEEP:LIST') )
      fprintf('  reply: OK    |');
   else
      fprintf('  reply: ERROR |');
      disp(reply);
   end
   %     check number of frequencies
   flushinput(ard);
   npts = query(ard, 'SWEEP:POINTS?\n', '%s','%i');
   if npts == length(f)
      fprintf('  #pts: OK    \n');
   else
      fprintf('  #pts: ERROR  (has %i, should be %i) \n', npts, length(f));
   end
end




fprintf(ard, cmd)
pause(1); while ard.BytesAvailable > 0; disp(fgetl(ard)); end

fprintf(ard, 'SWEEP:POINTS?')
pause(1); while ard.BytesAvailable > 0; disp(fgetl(ard)); end


fprintf(ard, 'PLL:REGVALS?')


% *******************************************************************************************************
% "RUN" COMMANDS

% panel sweep
fprintf(ard, 'RUN:PANELS');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

% frequency sweep
fprintf(ard, 'RUN:FREQ');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

% continuous panel sweep
fprintf(ard, 'CONT:PANELS');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

% frequency sweep
fprintf(ard, 'CONT:FREQ');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

% stop continuous sweep
%     option 1
fprintf(ard, 'STOP');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end
%     option 2
fprintf(ard, 'SINGLE'); 
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end



% *******************************************************************************************************
% "QUERY" COMMANDS

fprintf(ard, 'PLL:PFD?');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

fprintf(ard, 'TIME:DWELL?');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

fprintf(ard, 'TIME:PAUSE?');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

fprintf(ard, 'SWEEP:POINTS?');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end



% *******************************************************************************************************
% SET COMMANDS




fprintf(ard, 'TIME:DWELL 1234');
pause(0.5); if ard.BytesAvailable > 0; disp(fgetl(ard)); end

query(ard, 'TIME:DWELL?\n', '%s','%i')









  //     PFD frequency (set and query)
  else if (serial_cmd == "PLL:PFD") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    serial_parse_next_token(); // get value from buffer
    PFDFreq = serial_val; // update PFD frequency
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(PFDFreq);
  }
  else if (serial_cmd == "PLL:PFD?") {
    Serial.println(PFDFreq);
  } 
  //     dwell time (set and query)
  else if (serial_cmd == "TIME:DWELL") {
    serial_parse_next_token(); // get value from buffer
    sweep_dwell_time = serial_val; // update time
    check_sweep_timing(); // check new timing
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(sweep_dwell_time);
  } 
  else if (serial_cmd == "TIME:DWELL?") {
    Serial.println(sweep_dwell_time);
  } 
  //     update pause time between sweeps
  else if (serial_cmd == "TIME:PAUSE") {
    serial_parse_next_token(); // get value from buffer
    sweep_pause_time = serial_val; // update time
    check_sweep_timing(); // check new timing
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(sweep_pause_time);
  } 
  else if (serial_cmd == "TIME:PAUSE?") {
    Serial.println(sweep_pause_time);
  } 
  //     program standard sweep
  else if (serial_cmd == "SWEEP:DEFAULT") {
    pll_reset_sweep(); // reset old sweep
    pll_set_linear_sweep(SWEEP_DEFAULT_FSTART, SWEEP_DEFAULT_FSTEP,  SWEEP_DEFAULT_FSTOP);
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //     program linear sweep [min:step:max]
  else if (serial_cmd == "SWEEP:LIN") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    // get start, step, stop values from buffer
    int fstart = 0;
    int fstep = 0;
    int fstop = 0;
    if (serial_parse_next_token()) {
      fstart = serial_val;
    } 
    else {
      Serial.println("Start value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    if (serial_parse_next_token()) {
      fstep = serial_val;
    } 
    else {
      Serial.println("Step value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    if (serial_parse_next_token()) {
      fstop = serial_val;
    } 
    else {
      Serial.println("Stop value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    // send feedback
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd);
    Serial.print(" "); 
    Serial.print(fstart); 
    Serial.print(":"); 
    Serial.print(fstep); 
    Serial.print(":"); 
    Serial.println(fstop);
    // program sweep
    pll_set_linear_sweep(fstart, fstep, fstop);
  }
  //     program new list sweep
  else if (serial_cmd == "SWEEP:LIST") { 
    pll_reset_sweep(); // reset old sweep
    // send feedback
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd);
    // update sweep
    while (serial_parse_next_token()) { // while there are new values in the buffer ...
      pll_add_sweep_frequency(serial_val); // ... add them to the sweep
    }
  }
  //    select specific switch port (ports # start at 1)
  else if (serial_cmd == "SWITCH:SELECT") {
    serial_parse_next_token(); // get value from buffer
    switch_chain_selectfeed(serial_val); // switch on this feed
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(serial_val);
  } 
  //    get number of frequencies in sweep
  else if (serial_cmd == "SWEEP:POINTS?") {
    Serial.println(num_freq);
  } 
  //    get PLL register values
  else if (serial_cmd == "PLL:REGVALS?") {
    for(int i = 0; i < num_freq; i++) {
      Serial.print("|");
      Serial.print(((long)F2)    << 16 | ((long)F1)    << 8 | (long)F0, HEX);
      Serial.print("|");
      Serial.print(((long)R2)    << 16 | ((long)R1[i]) << 8 | (long)R0[i], HEX);
      Serial.print("|");
      Serial.print(((long)N2[i]) << 16 | ((long)N1[i]) << 8 | (long)N0[i], HEX);
      Serial.println(" ");
    }  
  } 
  //    unrecognized command; throw an error
  else {
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.print("Unrecognized command ");
    Serial.println(serial_cmd);
  }
  // reset command buffer
  serial_reset_cmd_interface();