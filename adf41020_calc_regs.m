% Calculates and returns decimal values for ADF41020 registers (F, R, N) for given RF output, input, 
% and PDF frequencies, and performs basic checks. Largely copied from ADI_PLL_Int-N (AD host software).
%
% RFout in MHz, REFin in MHz, PFDFreq in kHz
%
function [RFout_set, F, R, N] = adf41020_calc_regs(RFout, REFin, PFDFreq)

% save desired RFout 
RFout_des = RFout;


% *******************************************************************************************************
% basic device configuration; please refer to ADF41020 datasheet
%     WARNING: copied and adapted from Arduino script

Prescaler = uint32(1);
CPsetting1 = int8(3);
CPsetting2 = int8(3);
CPGain = int8(0);
CP3state = int8(0);
Fastlock = int8(0);
Timeout = int8(0);
PDPolarity = int8(0);
CounterReset = int8(0);
%LDP = int8(0);
Powerdown = int8(0);
%ABPW = int8(0);
%Sync = int8(0);
%Delay = int8(0);
Muxout = int8(0);
Testmodes = int8(1);



% *******************************************************************************************************
% private void BuildRegisters()

RFout = RFout / 4;

P = int32(2^Prescaler * 8);
R = int32(REFin * 1000 / PFDFreq);
N = int32(RFout * 1000 / PFDFreq);
B = int32(N / P);
A = int32(N - (B * P));

RFout_set = (((4 * (B * P + A)) * PFDFreq) / 1000);
           
if Fastlock == 2; Fastlock = Fastlock + 1; end
if Powerdown == 2; Powerdown = Powerdown + 1; end
    
R = uint32( 1 * 2^23 + 1 * 2^20 + uint32(Testmodes) * 2^16 + bitand(uint32(R), hex2dec('3FFF')) * 2^2 );
N = uint32( (uint32(CPGain) * 2^21) + bitand(uint32(B), hex2dec('1FFF')) * 2^8 + bitand(uint32(A), hex2dec('3F')) * 2^2 + 1 );
F = uint32( (uint32(Prescaler) * 2^22) + (uint32(CPsetting2) * 2^18) + (uint32(CPsetting1) * 2^15) + ...
   (uint32(Timeout) * 2^11) + (uint32(Fastlock) * 2^9) + (uint32(CP3state) * 2^8) + ...
   (uint32(PDPolarity) * 2^7) + (uint32(Muxout) * 2^4) + (uint32(Powerdown) * 2^3) + ...
   (uint32(CounterReset) * 2^2) + 2 );



% *******************************************************************************************************
% private void Limit_Check()

% AB counter input
if ((RFout / P) > 330)
   warning('Input to the AB counter should be less than 330 MHz.'); %#ok<*WNTAG>
end

% Check output = desired input
if (RFout_set ~= RFout_des)
   warning('Warning! Actually output frequency does not match desired frequency. Desired frequency not possible with chosen P, B and A.');
end

% P squared - P rule, A, B rules
if ((P * P - P) > N)
   warning('For continuously adjacant values of (N * REFin), at output, minimum N value is (P^2 - P).');
end
if (A > B)
   warning('B must be greater than or equal to A. Maybe change the prescaler...');
end
if ((B == 0) || (B == 1) || (B == 2))
   warning('B cannot equal 0, 1 or 2.');
end
if (B < A)
   warning('B must be greater than or equal to A.');
end

