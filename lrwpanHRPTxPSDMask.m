function lrwpanHRPTxPSDMask(wave, cfg)
% This is an internal example helper that may change any time.

%   Copyright 2021-2023 The MathWorks, Inc

sa = spectrumAnalyzer(SpectrumType='Power density', ChannelNames={'HPRF 802.15.4z'}, ...
  SampleRate=cfg.SampleRate, ...
  RBWSource='Property', RBW=1e6, YLimits = [-350 -40]);
sa(wave);

peakPRF = 499.2e6;
Fc065 = 0.65*peakPRF;
drop065 = -10; % dB
Fc08  =  0.8*peakPRF;
drop08 = -18; % dB

spectralMask = SpectralMaskSpecification;
spectralMask.EnabledMasks='Upper';
spectralMask.ReferenceLevel = 'Spectrum peak';
spectralMask.UpperMask = [-inf drop08; -Fc08 drop08; -Fc08 drop065; -Fc065 drop065; -Fc065 0;
                          Fc065 0; Fc065 drop065; Fc08 drop065; Fc08 drop08; inf drop08];
sa.SpectralMask = spectralMask;

assignin('caller', 'spectrumAnalyzer', sa);
