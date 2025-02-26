function lrwpanHRPPulseConformance(cfg)
% This is an internal example helper that may change any time.

%   Copyright 2021 The MathWorks, Inc

spc = cfg.SamplesPerPulse;

%% Create Butterworth pulse

% 1. Create a 4-th order Butterworth filter with 3-db bandwidth (cutoff) at 500 MHz
N = 4;
Fc = 500e6;
Fs = Fc*spc;
[b,a] = butter(N, Fc/Fs);

% 2. Pass an impulse to the Butterworth filter to create a Butterworth pulse
Tp = 2; % ns
NTp = 8;
xSpan = NTp*Tp; % ns
impulse = [1; zeros((NTp*spc-1), 1)];
pulse = filter(b, a, impulse);

% account for filter delay, center waveform at t=0
[~, idx] = max(pulse);
% butterPulse = circshift(butterPulse, round(length(butterPulse)/2)-idx);
len = length(pulse);
pulseCentered = [zeros(round(len/2)-idx, 1); pulse(1:(end-round(len/2)+idx))];


%% Cross-correlation check:

figXCorr = figure;

title('Cross-Correlation Compliance check')

subplot(1, 3, 1)
plot(-xSpan/2:(xSpan/(len-1)):xSpan/2, pulseCentered);
title('Butterworth Pulse')
xlabel('Time (ns)')
axis([-NTp NTp min(pulseCentered) max(pulseCentered)])


% 3. Create root-raised cosine pulse as per Sec 15.4.4
beta = 0.5;
t = -xSpan/2:(xSpan/(len-1)):xSpan/2;
tTp = t/Tp;
r = (4*beta/pi*sqrt(Tp)) * (cos((1+beta)*pi*tTp) + sin((1-beta)*pi*tTp)./(4*beta*tTp))./(1-(4*beta*tTp).^2);
r = r';

% avoid NaNs:
r(t==0) = (1+beta*(4/pi -1))/Tp;

% normalize:
r = r * 1/max(r);

subplot(1, 3, 2)
plot(t, r);
title('RRC Pulse')
axis([-NTp NTp min(r) max(r)])
xlabel('Time (ns)')

% 4. Calculate cross-correlation between Butterworth and Root-raised cosine pulses
x = xcorr(r, pulseCentered, 'normalized');
subplot(1, 3, 3)
len = length(x);
plot(-xSpan:(2*xSpan/(len-1)):xSpan, x);

T1 = 0.8;
T2 = 0.3;
hold on
plot([-xSpan/2 xSpan/2], [T1 T1], 'r-')
plot([-xSpan/2 xSpan/2], [T2 T2], 'r:')
plot([-xSpan/2 xSpan/2], [-T2 -T2], 'r:')
assert(any(x(round(end/2 +[-1 0 1]))>T1), 'Cross-correlation must be greater than 0.8 for 0 lag');
ind = find(abs(x)>T2);
assert(isequal(ind', min(ind):max(ind)), 'Cross-correlation greater than 0.3 is non-contiguous. That means some side-lobe is greater than 0.3');

title('Cross-Correlation')
axis([-NTp NTp -0.5 max(x)])
xlabel('Time (ns)')

%% Time-domain mask:


% Plot pulse together with the time-domain mask:
figMask = figure;
pulse = pulse/max(pulse);
xPulseStart = -1.25; % in Tp
xPulseEnd = 8; % in Tp
plot((xPulseStart:(xPulseEnd-xPulseStart)/(length(pulse)-1):xPulseEnd), pulse, 'b-o');
xMin = -3;
xMax = 9;
yMin = -0.8;
yMax = 1.1;
axis([xMin xMax yMin yMax])
t0 = 0;

% patches
a = 0.75;
darkRed = [200, 0, 0]/255;
patch([xMin xMin t0-1.25 t0-1.25 t0+1 t0+1 xMax xMax], [yMax 0.015 0.015 1 1 0.3 0.3 yMax], darkRed, 'FaceAlpha', a);
patch([xMin xMin t0 t0 t0+2 t0+2 xMax xMax], [yMin -0.015 -0.015 -0.5 -0.5 -0.3 -0.3 yMin], darkRed, 'FaceAlpha', a);


xlabel('Time (Tp)')
title ('Time Mask for Butterworth Pulse')

assignin('caller', 'figXCorrelation', figXCorr);
assignin('caller', 'figTimeDomainMask', figMask);