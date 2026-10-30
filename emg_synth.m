function [emg, t, muscle] = emg_synth(P, strength_mV, onWin, noise_mV, drift_mV, hum_mV, seed)
%EMG_SYNTH  Deterministic synthetic surface-EMG generator (a.c., centred 0 V).
%
%   [emg,t,muscle] = emg_synth(P, strength_mV, onWin, noise_mV, drift_mV, hum_mV, seed)
%
%   Band-limited Gaussian muscle process, amplitude-modulated by a smoothed
%   contraction gate (onWin, [s]), plus broadband noise, low-frequency
%   baseline drift, and 60 Hz mains interference. Returns emg [V] (a.c.; the
%   2.5 V rail is added inside the circuit model), time t [s], and the muscle
%   activation envelope (0..1). Falls back to a toolbox-free band-pass if the
%   Signal Processing Toolbox is unavailable.

if nargin < 7 || isempty(seed), seed = 7; end
rng(seed);
fs = P.fs;  t = (0:1/fs:P.Tstop-1/fs).';  N = numel(t);

muscle = zeros(N,1);
for k = 1:size(onWin,1)
    muscle = muscle + double(t>=onWin(k,1) & t<onWin(k,2));
end
kw = max(1,round(P.EMG.onsetTau*fs));
muscle = filter(ones(kw,1)/kw, 1, muscle);
muscle = min(max(muscle,0),1);

wn = randn(N,1);
Wn = P.EMG.band/(fs/2);  Wn = min(max(Wn,1e-4),0.999);
try
    [b,a] = butter(4, Wn, 'bandpass');  proc = filter(b,a,wn);
catch
    proc = local_bandpass(wn, P.EMG.band, fs);
end
proc = proc/std(proc);

emg = proc .* (strength_mV*1e-3) .* muscle;
emg = emg + noise_mV*1e-3.*randn(N,1);
emg = emg + drift_mV*1e-3.*sin(2*pi*P.EMG.drift_Hz*t);
emg = emg + hum_mV*1e-3.*sin(2*pi*P.EMG.hum_Hz*t);
end

function y = local_bandpass(x, band, fs)
ahp = exp(-2*pi*band(1)/fs); alp = exp(-2*pi*band(2)/fs);
yhp = zeros(size(x)); px=0; py=0;
for n=1:numel(x), yhp(n)=ahp*(py+x(n)-px); px=x(n); py=yhp(n); end
y = zeros(size(x)); py=0;
for n=1:numel(x), y(n)=(1-alp)*yhp(n)+alp*py; py=y(n); end
end
