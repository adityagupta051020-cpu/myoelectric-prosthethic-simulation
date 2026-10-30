function P = emg_params()
%EMG_PARAMS  Final parameter set for the EMG-controlled prosthetic-hand model.
%
%   P = emg_params() returns a struct with every circuit, control, and servo
%   parameter. Called with no output it also pushes the fields and a default
%   synthetic-EMG timeseries (emg_ts) into the base workspace so the Simulink
%   "From Workspace" source and block masks resolve.
%
%   FINAL / v2 — adds five-servo actuation dynamics (slew-rate-limited angle
%   response) so the model actuates realistic finger movement, and defaults
%   the comparator to the documented plain (no-hysteresis) behaviour.
%
%   All values come from the Group 7 BME 261 / BME 294L project documentation.
%   Engineering choices where the docs are incomplete/inconsistent are marked
%   "ASSUMPTION:".  Simulation only — NOT hardware validated.

% ---- solver / logging ----
P.fs      = 10e3;       % [Hz] model/sample rate
P.Tstop   = 12;        % [s]  simulation stop time
P.maxStep = 1/P.fs;    % [s]  solver max step (ode23tb)

% ---- power domains (op-amps & IA single-supply 0-5 V) ----
P.Vs_sig      = 5.0;   % [V] signal-processing positive rail
P.Vgnd        = 0.0;   % [V] signal ground
P.Vref_offset = 2.5;   % [V] mid-supply reference
P.Vs_servo    = 6.0;   % [V] SEPARATE servo rail (documentation only; not loaded in sim)
P.Iservo_stall= 2.5;   % [A] per-servo stall current (spec-class figure)
P.Nservo      = 5;     % five independent servo outputs

% ---- instrumentation amplifier (AD623) ----
P.IA.Rg   = 1300;                    % [ohm]
P.IA.G    = 1 + 100e3/P.IA.Rg;       % = 77.92 (report 77.9)
P.IA.Vref = P.Vref_offset;
P.IA.Gmax = 1000;                    % AD623 gain ceiling
% NOTE: on the physical board the AD623 realises this gain internally; the
% Falstad model uses the equivalent 3-op-amp IA (6x50k + 1.3k). The Simulink
% model represents it as a single gain block (identical transfer behaviour).

% ---- multiple-feedback band-pass (100-500 Hz) ----
P.BP.C   = 33e-9;
P.BP.R1  = 12e3;  P.BP.R3 = 96e3;  P.BP.R5 = 24e3;   % 96k = 80k+16k
P.BP.Req = (P.BP.R1*P.BP.R3)/(P.BP.R1+P.BP.R3);       % 10.667k
P.BP.num = [-1/(P.BP.R1*P.BP.C), 0];
P.BP.den = [1, 2/(P.BP.R5*P.BP.C), 1/(P.BP.Req*P.BP.R5*P.BP.C^2)];
P.BP.w0  = sqrt(1/(P.BP.Req*P.BP.R5*P.BP.C^2));
P.BP.f0  = P.BP.w0/(2*pi);                            % ~301 Hz
P.BP.BW  = (2/(P.BP.R5*P.BP.C))/(2*pi);               % ~402 Hz
P.BP.Q   = P.BP.f0/P.BP.BW;                           % 0.75
% NOTE: with Q=0.75 the true -3dB band is ~162-562 Hz (wider than 100-500).

% ---- precision full-wave rectifier ----
P.RECT.baseline = P.Vref_offset;

% ---- envelope 2nd-order low-pass (~10 Hz) ----
P.ENV.R = 16e3;  P.ENV.C = 1e-6;   % 1 uF (report "1 mF" is a typo)
a0 = 1/(P.ENV.R^2*P.ENV.C^2);  b1 = 2/(P.ENV.R*P.ENV.C);
P.ENV.num = a0;  P.ENV.den = [1, b1, a0];
P.ENV.fc  = sqrt(a0)/(2*pi);       % 9.95 Hz
P.ENV.Q   = sqrt(a0)/b1;           % 0.5

% ---- non-inverting amplifier ----
P.NI.Rf = 12e3;  P.NI.Rin = 1e3;
P.NI.G  = 1 + P.NI.Rf/P.NI.Rin;    % 13
P.G_total = P.IA.G*P.NI.G;         % ~1013
% ASSUMPTION: this stage is referenced to 2.5 V so it amplifies only the
% excursion above baseline (Vout = 2.5 + 13*(Venv-2.5)), otherwise it would
% saturate on the single 0-5 V rail.

% ---- comparator ----
P.CMP.Vth  = 3.0;  P.CMP.Vhi = 5.0;  P.CMP.Vlo = 0.0;
P.CMP.hyst = 0.0;  % documented plain comparator; firmware debounce gives stability
                   % (set e.g. 0.05 for optional Schmitt steadying)

% ---- firmware / control logic (Arduino Uno) ----
P.CTL.debounce   = 4.0;   % [s] 4000 ms buffer
P.CTL.angle_open = 5;     % [deg] servo.write(5) - hand OPEN
P.CTL.angle_init = 5;     % [deg] setup() position
% Per-finger CLOSED angles. Documented build used 65 deg for the single servo;
% kept 65 for all five (coordinated). Editable for a natural grip, e.g.
% [60 68 68 66 62] for thumb..pinky.  ASSUMPTION: uniform 65 deg by default.
P.CTL.angle_close_vec = [65 65 65 65 65];
P.CTL.pin_emg    = 11;                 % comparator -> digital input
P.CTL.pins_servo = [3 5 6 9 10];       % ASSUMPTION: PWM-capable Uno pins for 5 servos

% ---- servo actuation dynamics (finger movement) ----
% Hobby servos slew at a finite rate (~0.15-0.2 s per 60 deg). Modelled as a
% rate limiter on the commanded angle so the fingers physically travel rather
% than jumping instantly. A small first-order lag can be added if desired.
P.SERVO.slew_dps = 400;    % [deg/s] slew rate (60 deg in ~0.15 s)
P.SERVO.tau      = 0.02;   % [s] optional smoothing lag (used in offline check)

% ---- synthetic EMG source ----
P.EMG.strength_mV = 0.9;
P.EMG.band        = [20 450];
P.EMG.noise_mV    = 0.03;
P.EMG.drift_mV    = 0.30;   P.EMG.drift_Hz = 0.30;
P.EMG.hum_mV      = 0.30;   P.EMG.hum_Hz   = 60;
P.EMG.onWindows   = [2 3; 7 8];   % close at 2 s, open at 7 s
P.EMG.onsetTau    = 0.08;
P.EMG.level_weak     = 0.35;
P.EMG.level_moderate = 0.90;
P.EMG.level_strong   = 1.50;

% ---- default synthetic-EMG timeseries in base workspace ----
[emg,tv,mus] = emg_synth(P, P.EMG.strength_mV, P.EMG.onWindows, ...
                         P.EMG.noise_mV, P.EMG.drift_mV, P.EMG.hum_mV, 7);
emg_ts = timeseries(emg(:), tv(:), 'Name','emg_input');

if nargout == 0
    assignin('base','P',P);
    assignin('base','emg_ts',emg_ts);
    assignin('base','emg_muscle', timeseries(mus(:),tv(:)));
    f = fieldnames(P); for i=1:numel(f), assignin('base',f{i},P.(f{i})); end
    fprintf('emg_params (final): loaded P + emg_ts.\n');
    fprintf('  IA gain %.2f | BP f0 %.0f Hz Q %.2f | env %.2f Hz | x%.0f (total %.0f)\n',...
        P.IA.G,P.BP.f0,P.BP.Q,P.ENV.fc,P.NI.G,P.G_total);
    fprintf('  threshold %.1f V | debounce %.0f s | %d servos %d->%d deg, slew %d deg/s\n',...
        P.CMP.Vth,P.CTL.debounce,P.Nservo,P.CTL.angle_open,P.CTL.angle_close_vec(1),P.SERVO.slew_dps);
end
end
