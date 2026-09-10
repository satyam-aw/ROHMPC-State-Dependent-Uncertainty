function result = run_dynamic_bound_demo()
% Finite-grid certificate prototype, not a continuous-domain or MPC proof.
% Reuses a saved ROHMPC constant metric; performs no new SDP optimization.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
source = fullfile(root,'src','catkin_ws','src','mpc','mpc_solver','scripts','include','offline_computations','mpc-sdp');
input = fullfile(root,'src','catkin_ws','src','mpc_model_id_mismatch','data','model_mismatch_results','falcon_t.mat');
baseline = fullfile(root,'work','offline-sanity-20260910','offline_design.mat');
assert(isfile(baseline),'Run the baseline first; saved sanity-check result is missing.');
oldPath = path; cleanupPath = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(source);
assert(exist('casadi.SX','class')==8,'Add the Windows MATLAB CasADi distribution to the path.');
out = fullfile(here,'results',char(datetime('now','Format','yyyyMMdd_HHmmss')));
assert(~isfolder(out),'Result directory already exists.'); mkdir(out);
diary(fullfile(out,'run.log')); cleanupLog = onCleanup(@() diary('off')); %#ok<NASGU>
d = load(baseline); model = FalconModelT(input,true);
assert(model.nX==1 && model.nY==1,'Prototype assumes constant metric and gain.');
assert(norm(model.kd,'fro')==0,'Velocity independence check must be revisited if nominal drag changes.');
[n,ug,xg,Ag,Bg] = model.get_uxABXYgrid(false,false,1,[],[]);
P = full(d.P_delta(xg(:,1),ug(:,1))); P=(P+P')/2;
R = chol(P); K=full(d.K_delta(xg(:,1),ug(:,1))); L=d.L; C=model.C;
[~,Af,Bf] = model.get_fABfun();
baseRates = inf(2,1);
for j=1:n
    baseRates=min(baseRates,rates(Ag(:,:,j),Bg(:,:,j),R,K,L,C));
end
% Independent numerical audit: angle midpoints + all baseline thrust/rate vertices.
angles=linspace(model.att_min,model.att_max,3);
[pa,ta,ya]=ndgrid(angles,angles,angles);
combos=dec2bin(0:127,7)-'0';
limitsLow=[model.t_min*ones(4,1);model.wb_min*ones(3,1)];
limitsHigh=[model.t_max*ones(4,1);model.wb_max*ones(3,1)];
denseRates=inf(2,1); denseCount=0;
for a=1:numel(pa)
    for j=1:size(combos,1)
        q=limitsLow+(limitsHigh-limitsLow).*combos(j,:)';
        x=[zeros(3,1);pa(a);ta(a);ya(a);zeros(3,1);q(5:7)]; u=q(1:4);
        denseRates=min(denseRates,rates(full(Af(x,u)),full(Bf(x,u)),R,K,L,C));
        denseCount=denseCount+1;
    end
end
oldRng=rng; cleanupRng=onCleanup(@() rng(oldRng)); rng(17,'twister'); %#ok<NASGU>
randomCount=2000; randomRates=inf(2,1);
for j=1:randomCount
    x=model.x_lb+(model.x_ub-model.x_lb).*rand(model.nx,1);
    u=model.u_lb+(model.u_ub-model.u_lb).*rand(model.nu,1);
    randomRates=min(randomRates,rates(full(Af(x,u)),full(Bf(x,u)),R,K,L,C));
end
allRates=min([baseRates,denseRates,randomRates],[],2);
assert(all(allRates>0),'Saved metric failed positive-contraction audit. Do not propagate tubes.');
cert.P=P; cert.R=R; cert.K=K; cert.L=L;
cert.rhoObserver=0.95*allRates(1);
cert.rhoController=min(d.rho_c,0.95*allRates(2));
cert.gamma=norm(R*L*C/R,2);
Sv=zeros(3,model.nx); Sv(:,7:9)=eye(3);
cert.velocityMetricGain=norm(Sv/R,2);
cert.maximumSpeed=sqrt(3)*max(abs([model.v_min,model.v_max]));
cert.beta=0.25;
% Only channels injecting exclusively into acceleration are speed-scaled.
E=model.E; otherRows=setdiff(1:model.nx,7:9);
scaled=find(any(abs(E(7:9,:))>1e-12,1) & ~any(abs(E(otherRows,:))>1e-12,1));
assert(~isempty(scaled),'No acceleration-only disturbance channels found.');
rest=setdiff(1:model.nw,scaled);
assert(max(abs(model.w_min+model.w_max))<1e-10,'Expected centered residual disturbance box.');
cert.scaledChannels=scaled;
cert.processAcceleration=box_norm(R*E(:,scaled),model.w_min(scaled),model.w_max(scaled));
cert.processOther=box_norm(R*E(:,rest),model.w_min(rest),model.w_max(rest));
cert.noise=box_norm(R*L*model.F,model.eta_min,model.eta_max);
cert.globalForcing=cert.processOther+cert.processAcceleration+cert.noise;
cert.globalRadius=cert.globalForcing/cert.rhoObserver;
cert.globalTrackingRadius=(cert.gamma*cert.globalRadius+cert.noise)/cert.rhoController;
cert.baselineEpsilon=d.epsilon;
cert.baseRates=baseRates; cert.denseRates=denseRates; cert.randomRates=randomRates;
cert.baseCount=n; cert.denseCount=denseCount; cert.randomCount=randomCount;
cert.status='FINITE_POINT_AUDIT_ONLY: continuous-domain contraction coverage not proved';
cert.nonlinearAudit=check_error_dynamics(model,cert,23);
% Meaningful envelope checks: random true velocities inside metric tubes.
for j=1:2000
    s=0.1*rand; r=0.1*rand;
    zn=0.7*cert.maximumSpeed*rand; z=zeros(model.nx,1); z(7)=zn;
    a=randn(model.nx,1); b=randn(model.nx,1);
    delta=R\(s*rand*a/norm(a)); e=R\(r*rand*b/norm(b));
    actualSpeed=norm(Sv*(z+delta+e));
    if actualSpeed<=cert.maximumSpeed
        actualScale=cert.beta+(1-cert.beta)*(actualSpeed/cert.maximumSpeed)^2;
        [~,usedScale]=bound_rhs(0,[r;s],zn,cert);
        assert(actualScale<=usedScale+1e-12,'Tube speed envelope underestimated uncertainty.');
    end
end
% Global case has a known equilibrium, and provides a fair same-bound baseline.
assert(norm(bound_rhs(0,[cert.globalRadius;cert.globalTrackingRadius],cert.maximumSpeed,cert))<1e-10);
% Illustrate radius dynamics under a prescribed speed schedule, not plant simulation.
breaks=[0,8,16,32]; speed=[0,2.5,0];
y0=[cert.globalRadius;0]; t=[]; y=[]; zspeed=[];
options=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.02);
for k=1:3
    [tk,yk]=ode45(@(tt,yy) bound_rhs(tt,yy,speed(k),cert),[breaks(k),breaks(k+1)],y0,options);
    y0=yk(end,:)';
    if k>1, tk=tk(2:end); yk=yk(2:end,:); end
    t=[t;tk]; y=[y;yk]; zspeed=[zspeed;repmat(speed(k),numel(tk),1)]; %#ok<AGROW>
end
fixedR=cert.globalRadius*ones(size(t));
fixedS=cert.globalTrackingRadius*(1-exp(-cert.rhoController*t));
assert(all(y(:,1)>=-1e-10) && all(y(:,2)>=-1e-10));
assert(all(y(:,1)<=fixedR+1e-7) && all(y(:,2)<=fixedS+1e-7),'Comparison exceeded the global-envelope solution.');
scale=zeros(size(t)); certifiedSpeed=zeros(size(t));
for k=1:numel(t)
    [~,scale(k),certifiedSpeed(k)]=bound_rhs(t(k),y(k,:)',zspeed(k),cert);
end
traj=table(t,zspeed,certifiedSpeed,scale,y(:,1),fixedR,y(:,2),fixedS,...
    'VariableNames',{'time','nominal_speed','true_speed_upper','uncertainty_scale','dynamic_r','global_r','dynamic_s','global_s'});
writetable(traj,fullfile(out,'tube_comparison.csv'));
result.certificate=cert; result.trajectory=traj; result.outputDirectory=out;
save(fullfile(out,'dynamic_bound_result.mat'),'result');
fig=figure('Visible','off'); layout=tiledlayout(fig,3,1);
nexttile; plot(t,zspeed,t,certifiedSpeed,'--'); ylabel('Speed (m/s)'); legend('Prescribed nominal','Tube upper bound'); grid on;
nexttile; plot(t,y(:,1),t,fixedR,'--'); ylabel('Estimation radius'); legend('Dynamic r','Global-envelope r'); grid on;
nexttile; plot(t,y(:,2),t,fixedS,'--'); ylabel('Tracking radius'); xlabel('Time (s)'); legend('Dynamic s','Global-envelope s'); grid on;
title(layout,'Tube propagation illustration: not a closed-loop quadrotor simulation');
exportgraphics(fig,fullfile(out,'tube_comparison.png'),'Resolution',160); close(fig);
fprintf('Observer rates [original dense random]: %s\n',mat2str([baseRates(1),denseRates(1),randomRates(1)],8));
fprintf('Controller rates [original dense random]: %s\n',mat2str([baseRates(2),denseRates(2),randomRates(2)],8));
fprintf('Chosen rho_o=%g rho_c=%g gamma=%g velocity_gain=%g\n',cert.rhoObserver,cert.rhoController,cert.gamma,cert.velocityMetricGain);
fprintf('Forcing acceleration=%g other=%g noise=%g\n',cert.processAcceleration,cert.processOther,cert.noise);
fprintf('Global comparison r=%g; original SDP epsilon=%g (different constructions)\n',cert.globalRadius,d.epsilon);
fprintf('End dynamic r=%g s=%g; global r=%g s=%g\n',y(end,1),y(end,2),fixedR(end),fixedS(end));
fprintf('AUDIT_AND_ENVELOPE_TESTS_PASSED\n%s\nOUTPUT=%s\n',cert.status,out);
end

function rr=rates(A,B,R,K,L,C)
Ao=R*(A-L*C)/R; Ac=R*(A+B*K)/R;
rr=[-max(eig((Ao+Ao')/2));-max(eig((Ac+Ac')/2))];
end

function value=box_norm(G,lo,hi)
% Exact maximum of a convex norm over the vertices of this finite box.
n=numel(lo); if n==0, value=0; return; end
assert(n<=16,'Vertex enumeration too large for this prototype.');
bits=dec2bin(0:2^n-1,n)-'0';
vertices=lo(:)+(hi(:)-lo(:)).*bits';
value=max(vecnorm(G*vertices,2,1));
end