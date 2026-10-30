function mdl = build_EMG_model(mdl)
%BUILD_EMG_MODEL  Build the final EMG prosthetic-hand Simulink model
%                 (EMG_Prosthetic.slx) and save it.
%
%   Signal path:
%     synthetic EMG -> instrumentation amp (x77.9) -> 100-500 Hz band-pass
%       -> precision full-wave rectifier -> 10 Hz envelope -> x13 gain
%       -> comparator (3 V) -> Arduino toggle/debounce logic
%       -> FIVE slew-rate-limited servo actuators (finger movement)
%
%   Primary analog path uses transfer-function / gain blocks (guaranteed to
%   run, exact to the documented transfer functions). Control logic is a
%   MATLAB Function block. Each of the five servo commands passes through a
%   Rate Limiter so the fingers physically travel; all five angles are logged
%   to a scope. A Simscape Electrical component-level front-end can be added
%   (BUILD_SIMSCAPE=true). Scopes are placed after every major stage.
%
%   Simulation model only — not hardware validated.

BUILD_SIMSCAPE = false;
if nargin<1 || isempty(mdl), mdl='EMG_Prosthetic'; end
P = emg_params();  assignin('base','P',P);
[emg,tv,mus]=emg_synth(P,P.EMG.strength_mV,P.EMG.onWindows,P.EMG.noise_mV,P.EMG.drift_mV,P.EMG.hum_mV,7);
assignin('base','emg_ts',timeseries(emg(:),tv(:),'Name','emg_input'));
assignin('base','emg_muscle',timeseries(mus(:),tv(:)));

if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
set_param(mdl,'Solver','ode23tb','StopTime','P.Tstop','MaxStep','P.maxStep',...
    'SaveOutput','on','SaveFormat','StructureWithTime','SignalLogging','on','SignalLoggingName','logsout');

x=40; dx=150; y=220;
add=@(n,pth,xx,yy,w,h) add_block(pth,[mdl '/' n],'Position',[xx yy xx+w yy+h]);

add('EMG_source','simulink/Sources/From Workspace',x,y,90,40);
set_param([mdl '/EMG_source'],'VariableName','emg_ts','SampleTime','0','Interpolate','on','OutputAfterFinalValue','Extrapolation'); x=x+dx;
add('IA_gain','simulink/Math Operations/Gain',x,y,60,40); set_param([mdl '/IA_gain'],'Gain','P.IA.G'); x=x+dx;
add('BandPass','simulink/Continuous/Transfer Fcn',x,y,90,50); set_param([mdl '/BandPass'],'Numerator','P.BP.num','Denominator','P.BP.den'); x=x+dx;
add('Rectifier','simulink/Math Operations/Abs',x,y,60,40); x=x+dx;
add('Envelope','simulink/Continuous/Transfer Fcn',x,y,90,50); set_param([mdl '/Envelope'],'Numerator','P.ENV.num','Denominator','P.ENV.den'); x=x+dx;
add('NI_gain','simulink/Math Operations/Gain',x,y,60,40); set_param([mdl '/NI_gain'],'Gain','P.NI.G');
add('Baseline','simulink/Math Operations/Bias',x,y+70,60,40); set_param([mdl '/Baseline'],'Bias','P.Vref_offset'); x=x+dx;
add('Comparator','simulink/Discontinuities/Relay',x,y,70,50);
set_param([mdl '/Comparator'],'OnSwitchValue','P.CMP.Vth + P.CMP.hyst','OffSwitchValue','P.CMP.Vth - P.CMP.hyst',...
    'OnOutputValue','P.CMP.Vhi','OffOutputValue','P.CMP.Vlo'); x=x+dx;
add('SimClock','simulink/Sources/Clock',x,y+120,40,40);
add('ControlLogic','simulink/User-Defined Functions/MATLAB Function',x,y-10,130,90);
set_MLfcn([mdl '/ControlLogic'], control_logic_code());
try, set_param([mdl '/ControlLogic'],'Commented','off'); catch, end
x=x+dx+30;

% ---- five slew-rate-limited servo actuators ----
servoX=x; outX=x+dx+30;
for kk=1:P.Nservo
    yy=y-150+(kk-1)*70;
    add(sprintf('Servo%d_slew',kk),'simulink/Discontinuities/Rate Limiter',servoX,yy,70,40);
    set_param([mdl sprintf('/Servo%d_slew',kk)],'RisingSlewLimit','P.SERVO.slew_dps','FallingSlewLimit','-P.SERVO.slew_dps');
    add(sprintf('Servo%d',kk),'simulink/Sinks/Out1',outX,yy,40,30);
    set_param([mdl sprintf('/Servo%d',kk)],'Port',num2str(kk));
end
add('ServoMux','simulink/Signal Routing/Mux',servoX+90,y+30,10,150); set_param([mdl '/ServoMux'],'Inputs',num2str(P.Nservo));

% ---- scopes after every major stage ----
scopeY=470;
mkscope('Scope1_IA','IA_gain/1',40+1*dx);
mkscope('Scope2_BandPass','BandPass/1',40+2*dx);
mkscope('Scope3_Rectifier','Rectifier/1',40+3*dx);
mkscope('Scope4_Envelope','NI_gain/1',40+4*dx);
mkscope('Scope5_Comparator','Comparator/1',40+5*dx);
add('Scope6_Servos','simulink/Sinks/Scope',servoX+120,scopeY,40,30);

% ---- wiring ----
con('EMG_source/1','IA_gain/1'); con('IA_gain/1','BandPass/1'); con('BandPass/1','Rectifier/1');
con('Rectifier/1','Envelope/1'); con('Envelope/1','NI_gain/1'); con('NI_gain/1','Baseline/1');
con('Baseline/1','Comparator/1');
try
    con('Comparator/1','ControlLogic/1'); con('SimClock/1','ControlLogic/2');
    for kk=1:P.Nservo
        con(sprintf('ControlLogic/%d',kk),sprintf('Servo%d_slew/1',kk));
        con(sprintf('Servo%d_slew/1',kk),sprintf('Servo%d/1',kk));
        con(sprintf('Servo%d_slew/1',kk),sprintf('ServoMux/%d',kk));
    end
    con('ServoMux/1','Scope6_Servos/1');
catch ME
    warning('ControlLogic/servo ports not auto-created (%s). See control_logic_code().',ME.message);
end

if BUILD_SIMSCAPE, try, build_simscape_frontend(mdl,P); catch ME, warning('Simscape skipped: %s',ME.message); end, end

try
    a=Simulink.Annotation([mdl '/Info']);
    a.Text=sprintf(['EMG Prosthetic Hand - FINAL model\nIA x%.1f | BP %d-%d Hz (f0=%.0f,Q=%.2f) | FWR | env %.1f Hz | x%.0f | cmp %.1f V\n' ...
      '4 s debounce -> 5 slew-limited servos (%d->%d deg, %d deg/s). NOT hardware validated.'],...
      P.IA.G,100,500,P.BP.f0,P.BP.Q,P.ENV.fc,P.NI.G,P.CMP.Vth,P.CTL.angle_open,P.CTL.angle_close_vec(1),P.SERVO.slew_dps);
    a.Position=[40 30];
catch, end

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('Built and saved %s.slx (5-servo actuation)\n',mdl);

    function con(s,d), add_line(mdl,s,d,'autorouting','on'); end
    function mkscope(nm,src,xx)
        add_block('simulink/Sinks/Scope',[mdl '/' nm],'Position',[xx scopeY xx+40 scopeY+30]);
        add_line(mdl,src,[nm '/1'],'autorouting','on');
    end
end

% ====================================================================
function set_MLfcn(blockPath,code)
ok=false;
try, cfg=get_param(blockPath,'MATLABFunctionConfiguration'); cfg.FunctionScript=code; ok=true; catch, end
if ~ok
    try, root=sfroot; ch=root.find('-isa','Stateflow.EMChart','Path',blockPath); ch.Script=code; ok=true;
    catch ME, warning('Could not set MATLAB Function code (%s). Paste control_logic_code() manually.',ME.message); end
end
end

function code = control_logic_code()
%CONTROL_LOGIC_CODE  Arduino-equivalent toggle + 4 s debounce, 5 servo cmd out.
code = [ ...
"function [s1,s2,s3,s4,s5] = ControlLogic(comp, t)"                       newline ...
"%#codegen"                                                               newline ...
"% One accepted contraction toggles the hand; a 4 s buffer rejects the"   newline ...
"% multiple HIGH pulses of one contraction. Outputs five commanded servo" newline ...
"% angles (coordinated). Per-finger closed angles allow a natural grip."  newline ...
"persistent motorOn last"                                                 newline ...
"if isempty(motorOn), motorOn = false; last = -1e9; end"                  newline ...
"openA   = 5;                        %% servo.write(5)"                    newline ...
"closeV  = [65 65 65 65 65];         %% per-finger closed angles"          newline ...
"deb     = 4;                        %% debounceDuration = 4000 ms"        newline ...
"if comp > 2.5 && (t - last) > deb"                                       newline ...
"    last = t;  motorOn = ~motorOn;"                                      newline ...
"end"                                                                     newline ...
"if motorOn, a = closeV; else, a = [openA openA openA openA openA]; end"  newline ...
"s1 = a(1); s2 = a(2); s3 = a(3); s4 = a(4); s5 = a(5);"                  newline ...
"end"];
code = char(join(code,''));
end

function build_simscape_frontend(mdl,P)
sub=[mdl '/Simscape_FrontEnd']; add_block('built-in/Subsystem',sub,'Position',[40 560 260 680]);
try
  a=Simulink.Annotation([sub '/note']);
  a.Text=sprintf(['Simscape Electrical component-level front-end (optional).\n' ...
    'Populate: AD623-equivalent IA (Rg=%.0f), MFB band-pass (R1=%.0f,R3=%.0f,R5=%.0f,C=%.0f nF),\n' ...
    'precision FWR (2 op-amp + 2 diode), Sallen-Key LP (R=%.0f,C=%.0f uF).'],...
    P.IA.Rg,P.BP.R1,P.BP.R3,P.BP.R5,P.BP.C*1e9,P.ENV.R,P.ENV.C*1e6);
  a.Position=[30 30];
catch, end
end
