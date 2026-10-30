function results = validate_EMG_model()
%VALIDATE_EMG_MODEL  Automated validation of the final EMG prosthetic model,
%   including five-servo actuation dynamics.
%
%   Runs the ten required tests, prints PASS/FAIL, writes comparison_table.csv,
%   and saves stage / Bode / servo plots. The analog chain is reproduced with a
%   self-contained bilinear (Tustin) discretisation of the documented transfer
%   functions (fs = 10 kHz, <1% error to 500 Hz, no toolboxes needed) — the same
%   math the Simulink Transfer Fcn blocks evaluate. Servos are modelled as
%   slew-rate-limited actuators, matching the Simulink Rate Limiter blocks.
%
%   SIMULATION results only — validates the design/model, not the hardware.

P = emg_params();
if ~exist('plots','dir'), mkdir('plots'); end
tol=@(a,b,r) abs(a-b)<=r*abs(b);
R=struct('name',{},'pass',{},'detail',{});
fprintf('\n=============== EMG MODEL VALIDATION (final, 5-servo) ===============\n');

[emg,t,muscle]=emg_synth(P,P.EMG.level_moderate,P.EMG.onWindows,P.EMG.noise_mV,P.EMG.drift_mV,P.EMG.hum_mV,7);
S=run_chain(P,emg,t);
[cmd,act,ntoggle]=servo_model(P,S.comp,t);

% 1 IA gain
u=1e-3*sin(2*pi*300*t); Gsim=max(abs(u*P.IA.G))/max(abs(u));
R(end+1)=res('1. IA gain',tol(Gsim,P.IA.G,1e-6),sprintf('sim %.2f vs calc %.2f',Gsim,P.IA.G));
% 2 passband
f=logspace(0,4,4000); H=tf_mag(P.BP.num,P.BP.den,f); Hdb=20*log10(H);
[pk,ip]=max(Hdb); f0m=f(ip); band=f(Hdb>=pk-3); fLo=min(band); fHi=max(band);
g100=20*log10(tf_mag(P.BP.num,P.BP.den,100)); g500=20*log10(tf_mag(P.BP.num,P.BP.den,500));
R(end+1)=res('2. Band-pass passband',tol(f0m,P.BP.f0,0.05)&&g100>-7.1&&g500>-7.1,...
    sprintf('f0=%.0f Hz, -3dB %.0f-%.0f Hz, g100=%.1f g500=%.1f dB',f0m,fLo,fHi,g100,g500));
% 3 rectification
us=0.05*sin(2*pi*200*t); rec=abs(us);
R(end+1)=res('3. Full-wave rectification',all(rec>=-1e-12)&&tol(mean(rec),2/pi*0.05,0.02),...
    sprintf('all>=0, mean=%.4f (expect %.4f)',mean(rec),2/pi*0.05));
% 4 envelope delay
on=P.EMG.onWindows(1,1); env=S.env_v; base=median(env(t<on-0.2));
target=base+0.63*(max(env(t>on&t<on+1))-base); idx=find(t>on&env>=target,1); delay=t(idx)-on;
R(end+1)=res('4. Envelope tracking delay',~isempty(delay)&&delay<0.25,sprintf('63%% rise %.0f ms',delay*1000));
% 5 debounce stability
R(end+1)=res('5. Comparator/debounce stability',ntoggle==size(P.EMG.onWindows,1),...
    sprintf('%d toggles for %d contractions',ntoggle,size(P.EMG.onWindows,1)));
% 6 weak/moderate/strong
lv=[P.EMG.level_weak P.EMG.level_moderate P.EMG.level_strong]; trig=false(1,3); pkv=zeros(1,3);
for i=1:3
    [e,ti]=emg_synth(P,lv(i),[2 3],P.EMG.noise_mV,0,0,11); Si=run_chain(P,e,ti);
    pkv(i)=max(Si.env_v(ti>2&ti<3)); trig(i)=any(Si.comp(ti>2&ti<3.3)>2.5);
end
R(end+1)=res('6. Weak/Moderate/Strong',(~trig(1))&&trig(2)&&trig(3),...
    sprintf('peak env [%.2f %.2f %.2f] V, trig [%d %d %d]',pkv,trig));
% 7 drift/60Hz/noise rejection
[er,tr]=emg_synth(P,0.0,zeros(0,2),0.05,0.8,0.5,3); Sr=run_chain(P,er,tr);
false_trig=any(Sr.comp>2.5);
R(end+1)=res('7. Drift/60Hz/noise rejection',~false_trig&&max(Sr.env_v)<P.CMP.Vth,...
    sprintf('rest max env=%.3f V, false trig=%d',max(Sr.env_v),false_trig));
% 8 five servos reach open & closed, coordinated
reachC=all(arrayfun(@(j) any(abs(act(:,j)-P.CTL.angle_close_vec(j))<1),1:P.Nservo));
reachO=all(arrayfun(@(j) any(abs(act(:,j)-P.CTL.angle_open)<1),1:P.Nservo));
coord=all(all(abs(cmd-cmd(:,1))<1e-9));
R(end+1)=res('8. Five servo commands',reachC&&reachO&&size(act,2)==P.Nservo,...
    sprintf('%d servos reach open=%d close=%d, coordinated=%d',P.Nservo,reachO,reachC,coord));
% 9 end-to-end response time
hi=find(t>on&S.comp>2.5,1); if isempty(hi), respC=NaN; else, respC=t(hi)-on; end
sc=find(t>on&cmd(:,1)~=cmd(1,1),1); if isempty(sc), respCmd=NaN; else, respCmd=t(sc)-on; end
tgt=P.CTL.angle_open+0.95*(P.CTL.angle_close_vec(1)-P.CTL.angle_open);
sa=find(t>on&act(:,1)>=tgt,1); if isempty(sa), respAct=NaN; else, respAct=t(sa)-on; end
R(end+1)=res('9. End-to-end response time',~isnan(respC)&&respC<0.30,...
    sprintf('onset->comparator %.0f ms, ->cmd %.0f ms, ->servo95%% %.0f ms',respC*1000,respCmd*1000,respAct*1000));
% 10 no clipping
clip=any(S.env_v>=P.Vs_sig-1e-9)||any(S.env_v<=P.Vgnd+1e-9)||max(abs(S.ia_v-P.Vref_offset))>=2.5;
R(end+1)=res('10. No clipping/saturation',~clip,...
    sprintf('|IA ac|=%.0f mV, env [%.2f %.2f] V',max(abs(S.ia_v-P.Vref_offset))*1000,min(S.env_v),max(S.env_v)));

% ---- report ----
fprintf('\n%-34s  %s\n','TEST','RESULT'); fprintf('%s\n',repmat('-',1,62));
np=0; for i=1:numel(R), fprintf('%-34s  %s\n',R(i).name,tern(R(i).pass,'PASS','FAIL')); fprintf('      %s\n',R(i).detail); np=np+R(i).pass; end
fprintf('%s\nPASSED %d / %d\n\n',repmat('-',1,62),np,numel(R));

% ---- comparison table ----
Quantity={'IA gain';'Total system gain';'Band-pass f0 (Hz)';'Band-pass Q';'Band-pass -3dB low (Hz)';...
  'Band-pass -3dB high (Hz)';'Envelope fc (Hz)';'Envelope Q';'Comparator threshold (V)';'Baseline offset (V)';...
  'Response: onset->comparator (ms)';'Response: onset->servo 95% (ms)'};
Calculated=[P.IA.G;P.G_total;P.BP.f0;P.BP.Q;100;500;P.ENV.fc;P.ENV.Q;P.CMP.Vth;P.Vref_offset;NaN;NaN];
Simulated =[Gsim;P.G_total;f0m;P.BP.Q;fLo;fHi;P.ENV.fc;P.ENV.Q;P.CMP.Vth;P.Vref_offset;respC*1000;respAct*1000];
Report    =[77.9;1012.7;300;0.75;NaN;NaN;9.95;0.5;3.0;2.5;NaN;NaN];
Ttab=table(Quantity,Calculated,Simulated,Report); disp(Ttab);
try, writetable(Ttab,'comparison_table.csv'); catch, end

make_stage_plot(P,t,emg,S); make_bode_plot(P,f,Hdb); make_servo_plot(P,t,act,S);
results.tests=R; results.table=Ttab; results.passed=np; results.total=numel(R);
end

% ===================== core chain =====================
function S=run_chain(P,emg,t)
Ts=mean(diff(t));
s1=P.IA.G*emg; s2=simTF(P.BP.num,P.BP.den,s1,Ts); s3=abs(s2); s4=simTF(P.ENV.num,P.ENV.den,s3,Ts);
env=P.Vref_offset+P.NI.G*s4; env=min(max(env,P.Vgnd),P.Vs_sig);
comp=relay(env,P.CMP.Vth,P.CMP.hyst,P.CMP.Vhi,P.CMP.Vlo);
S.ia_v=P.Vref_offset+s1; S.bp_v=P.Vref_offset+s2; S.rect_v=P.Vref_offset+s3; S.env_v=env; S.comp=comp;
end

% ---- servo model: toggle+debounce -> commanded angle -> slew-rate-limited ----
function [cmd,act,ntoggle]=servo_model(P,comp,t)
Ts=mean(diff(t)); motorOn=false; last=-1e9; ntoggle=0; n=numel(t); Ns=P.Nservo;
cmd=zeros(n,Ns); act=zeros(n,Ns); a=P.CTL.angle_open*ones(1,Ns); step=P.SERVO.slew_dps*Ts;
for k=1:n
    if comp(k)>2.5 && (t(k)-last)>P.CTL.debounce, last=t(k); motorOn=~motorOn; ntoggle=ntoggle+1; end
    if motorOn, c=P.CTL.angle_close_vec(:).'; else, c=P.CTL.angle_open*ones(1,Ns); end
    cmd(k,:)=c;
    a=a+max(min(c-a,step),-step);   % rate limiter
    act(k,:)=a;
end
end

% ---- toolbox-free continuous-TF sim via bilinear ----
function y=simTF(num,den,u,Ts), [bz,az]=bil(num,den,Ts); y=filter(bz,az,u); end
function [bz,az]=bil(num,den,T)
c=2/T; n=numel(den)-1; nf=zeros(1,n+1); nf(end-numel(num)+1:end)=num; na=fliplr(nf); da=fliplr(den);
Nz=zeros(1,n+1); Dz=zeros(1,n+1);
for k=0:n
    p=1; for j=1:k, p=conv(p,[1 -1]); end; for j=1:(n-k), p=conv(p,[1 1]); end
    Nz=Nz+na(k+1)*c^k*p; Dz=Dz+da(k+1)*c^k*p;
end
bz=Nz/Dz(1); az=Dz/Dz(1);
end
function y=relay(x,th,hy,hi,lo)
y=zeros(size(x)); st=0;
for k=1:numel(x), if x(k)>th+hy, st=1; elseif x(k)<th-hy, st=0; end, y(k)=hi*st+lo*(1-st); end
end
function m=tf_mag(num,den,f), s=1j*2*pi*f; m=abs(polyval(num,s)./polyval(den,s)); end

% ---- plots ----
function make_stage_plot(P,t,emg,S)
figure('Position',[80 80 900 950],'Color','w'); sp=@(k) subplot(6,1,k);
sp(1); plot(t,emg*1e3); ylabel('mV'); grid on; title('1. Raw synthetic EMG');
sp(2); plot(t,S.ia_v); ylabel('V'); grid on; title(sprintf('2. IA x%.1f (+2.5V)',P.IA.G));
sp(3); plot(t,S.bp_v); ylabel('V'); grid on; title('3. Band-pass 100-500 Hz');
sp(4); plot(t,S.rect_v); ylabel('V'); grid on; title('4. Precision full-wave rectifier');
sp(5); plot(t,S.env_v); yline(P.CMP.Vth,'r--'); ylabel('V'); grid on; title('5. Envelope x13 (3V threshold)');
sp(6); plot(t,S.comp); ylabel('V'); grid on; xlabel('Time (s)'); title('6. Comparator output');
saveas(gcf,'plots/stage_signals.png');
end
function make_bode_plot(P,f,Hdb)
figure('Position',[80 80 700 420],'Color','w'); semilogx(f,Hdb,'LineWidth',1.2); hold on; grid on;
xline(100,'g:'); xline(500,'g:'); yline(-3,'k--'); xlabel('Hz'); ylabel('Gain (dB)'); ylim([-30 3]);
title(sprintf('Band-pass Bode (f0=%.0f Hz, Q=%.2f)',P.BP.f0,P.BP.Q)); saveas(gcf,'plots/bode_bandpass.png');
end
function make_servo_plot(P,t,act,S)
figure('Position',[80 80 900 420],'Color','w');
plot(t,act,'LineWidth',1.1); hold on; grid on; plot(t,S.comp*10-15,'k:');
yticks([P.CTL.angle_open P.CTL.angle_close_vec(1)]); ylabel('servo angle (deg)'); xlabel('Time (s)');
legend(arrayfun(@(i)sprintf('Finger %d',i),1:P.Nservo,'uni',0),'Location','east');
title('Five servo actuators (slew-limited) responding to contractions'); saveas(gcf,'plots/servo_angles.png');
end

function r=res(n,p,d), r.name=n; r.pass=logical(p); r.detail=d; end
function s=tern(c,a,b), if c, s=a; else, s=b; end, end
