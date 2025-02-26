function hZoomToBPMBPSKSymbols(fig, cfg)
% This is an internal example helper that may change any time.

%   Copyright 2021 The MathWorks, Inc.

ind = lrwpanHRPFieldIndices(cfg);
phrEnd = ind.PHR(end) + cfg.SamplesPerPulse; % also account for group delay

ax = get(fig,'CurrentAxes');

xLim = phrEnd + [-1 1].*cfg.ChipsPerSymbol*cfg.SamplesPerPulse;
set(ax, 'XLim', xLim*1e6/cfg.SampleRate); % in ms

hold(ax, 'on')
plot(ax, [phrEnd phrEnd], get(gca, 'YLim'), 'k-')

% PHR
for idx = 1:cfg.BurstsPerSymbol(1)
  x = xLim(1) + idx*(phrEnd-xLim(1))/cfg.BurstsPerSymbol(1);
  y = get(ax, 'YLim');
  plot(ax, [x x], y, 'k:')
end

% Payload
for idx = 1:cfg.BurstsPerSymbol(end)
  x = phrEnd+1 + idx*(xLim(end)-phrEnd)/cfg.BurstsPerSymbol(end);
  y = get(ax, 'YLim');
  plot(ax, [x x], y, 'k:')
end

hold(ax, 'off')

legend(ax, 'SYNC', 'SFD', 'STS', 'PHR', 'Payload');
legend(ax, 'Location', 'best')
title(ax, 'Single PHR & Single Payload Symbol in BPRF mode');

