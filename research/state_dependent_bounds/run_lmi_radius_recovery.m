function result=run_lmi_radius_recovery()
% Recover the original squared-radius inequality, then shrink its process envelope.
% Numerical LMI verification retains the original finite-domain coverage limitation.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
source=fullfile(root,'src','catkin_ws','src','mpc','mpc_solver','scripts','include','offline_computations','mpc-sdp');
oldPath=path; pathCleanup=onCleanup(@() path(oldPath)); addpath(source); %#ok<NASGU>
input=fullfile(root,'src','catkin_ws','src','mpc_model_id_mismatch','data','model_mismatch_results','falcon_t.mat');
d=load(fullfile(root,'work','offline-sanity-20260910','offline_design.mat'));
m=FalconModelT(input,true); assert(m.nX==1 && m.nY==1);
out=fullfile(here,'results',['lmi_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
assert(~isfolder(out)); mkdir(out); diary(fullfile(out,'run.log')); logCleanup=onCleanup(@() diary('off')); %#ok<NASGU>
[n,ug,xg,Ag,Bg]=m.get_uxABXYgrid(false,true,1,[],[]);
X=d.X_arr; P=full(d.P_delta(xg(:,1),ug(:,1))); P=(P+P')/2; T=chol(P);
assert(norm(P*X-eye(m.nx),'fro')<1e-7,'Metric conversion mismatch.');
W=(d.W_bar+d.W_bar')/2; H=(d.H1_bar+d.H1_bar')/2;
Q=W(1:m.nx,1:m.nx); h=W(1:m.nx,end); c=W(end,end);
[U,cholFlag]=chol(Q); assert(cholFlag==0,'Process envelope needs a singular Schur-complement treatment.');
[w,nw]=m.get_wgrid(); [eta,neta]=m.get_etagrid();
scaled=find(any(abs(m.E(7:9,:))>1e-12,1) & ~any(abs(m.E(setdiff(1:m.nx,7:9),:))>1e-12,1));
assert(~isempty(scaled)); assert(max(abs(m.w_min+m.w_max))<1e-10);
beta=0.25; localw=w; localw(scaled,:)=beta*localw(scaled,:);
g1=max(sum((U'\(m.E*w-h)).^2,1));
gb=max(sum((U'\(m.E*localw-h)).^2,1));
assert(g1>=gb-1e-10,'Nested process sets gave a nonmonotone envelope.');
% Retain 1% of the available improvement as a numerical cushion.
saving=0.99*max(0,g1-gb);
q0=d.lambda_epsilon*d.epsilon^2;
assert(q0-saving>0,'Chosen local forcing is nonpositive; inspect conditioning.');
params.lambda=d.lambda_epsilon; params.epsilon=d.epsilon; params.q0=q0;
params.beta=beta; params.saving=saving;
params.rho=d.rho_c; params.gamma=norm(T*d.L*m.C/T,2);
params.noise=max(vecnorm(T*d.L*m.F*eta,2,1));
Sv=zeros(3,m.nx); Sv(:,7:9)=eye(3);
params.cv=norm(Sv/T,2); params.vmax=sqrt(3)*max(abs([m.v_min,m.v_max]));
tol=d.sol_check_tol;
% Explicit eigenvalue audit of all three original LMI families.
processResidual=-inf; noiseResidual=-inf; dynamicsResidual=-inf; trackingResidual=-inf;
for j=1:nw
    b=m.E*w(:,j); S=[zeros(m.nx),b;b',0];
    processResidual=max(processResidual,max(eig((S-W+S'-W')/2)));
end
for j=1:neta
    b=-d.L*m.F*eta(:,j); S=[zeros(m.nx),b;b',0];
    noiseResidual=max(noiseResidual,max(eig((S-H+S'-H')/2)));
end
for j=1:n
    Ao=Ag(:,:,j)-d.L*m.C;
    Z=W+H+blkdiag(X*Ao'+Ao*X+params.lambda*X,-q0);
    dynamicsResidual=max(dynamicsResidual,max(eig((Z+Z')/2)));
    K=full(d.K_delta(xg(:,j),ug(:,j))); Ac=Ag(:,:,j)+Bg(:,:,j)*K;
    Zc=Ac*X+X*Ac'+2*params.rho*X;
    trackingResidual=max(trackingResidual,max(eig((Zc+Zc')/2)));
end
assert(max([processResidual,noiseResidual,dynamicsResidual,trackingResidual])<=tol,'Original LMI residual exceeds its saved tolerance.');
% Audit the new envelope with all disturbance vertices at 31 scale values.
localSchurResidual=-inf;
for a=linspace(beta,1,31)
    wa=w; wa(scaled,:)=a*wa(scaled,:);
    exact=max(sum((U'\(m.E*wa-h)).^2,1));
    localBottom=c-saving*(1-a)/(1-beta);
    localSchurResidual=max(localSchurResidual,exact-localBottom);
end
assert(localSchurResidual<=tol,'Local envelope failed Schur-complement audit.');
% Global squared-radius ODE must recover the ORIGINAL epsilon exactly.
opts=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.025);
[tcheck,Rcheck]=ode45(@(t,R) -params.lambda*R+q0,[0,20],0.2*d.epsilon^2,opts);
Rexact=d.epsilon^2+(0.2*d.epsilon^2-d.epsilon^2)*exp(-params.lambda*tcheck);
assert(max(abs(Rcheck-Rexact))<1e-10);
assert(abs(sqrt(q0/params.lambda)-d.epsilon)<1e-12);
% Direct nonlinear derivative tests at interior states; this is falsification, not proof.
oldRng=rng; rngCleanup=onCleanup(@() rng(oldRng)); rng(41,'twister'); %#ok<NASGU>
[f,~,~]=m.get_fABfun(); maxExcess=-inf; accepted=0;
K=full(d.K_delta(xg(:,1),ug(:,1)));
for j=1:2000
    z=m.x_lb+(m.x_ub-m.x_lb).*(0.2+0.6*rand(m.nx,1));
    vn=m.u_lb+(m.u_ub-m.u_lb).*(0.3+0.4*rand(m.nu,1));
    vec=randn(m.nx,1); e=T\((0.001+0.08*rand)*vec/norm(vec));
    vec=randn(m.nx,1); delta=T\((0.001+0.008*rand)*vec/norm(vec));
    xhat=z+delta; x=xhat+e; u=vn+K*delta;
    if any(x<m.x_lb | x>m.x_ub) || any(xhat<m.x_lb | xhat>m.x_ub) || any(u<m.u_lb | u>m.u_ub), continue; end
    r=norm(T*e); s=norm(T*delta);
    upper=min(params.vmax,norm(z(7:9))+params.cv*(r+s));
    aupper=beta+(1-beta)*(upper/params.vmax)^2;
    atrue=beta+(1-beta)*(norm(x(7:9))/params.vmax)^2;
    assert(atrue<=aupper+1e-12);
    wj=w(:,randi(nw)); wj(scaled)=atrue*wj(scaled); etaj=eta(:,randi(neta));
    edot=full(f(x,u))-full(f(xhat,u))-d.L*m.C*e+m.E*wj-d.L*m.F*etaj;
    forcing=q0-saving*(1-aupper)/(1-beta);
    maxExcess=max(maxExcess,2*e'*P*edot-(-params.lambda*r^2+forcing));
    accepted=accepted+1;
end
assert(accepted>=500 && maxExcess<=1e-8,'Direct nonlinear error inequality failed.');
% Same metric/gains/initial radius/noise for fixed and local propagation.
breaks=[0,8,16,32]; speeds=[0,2.5,0]; y0=[d.epsilon^2;0]; tt=[]; yy=[]; vv=[];
for k=1:3
    [tk,yk]=ode45(@(t,y) lmi_rhs(y,speeds(k),params),[breaks(k),breaks(k+1)],y0,opts);
    y0=yk(end,:)'; if k>1, tk=tk(2:end); yk=yk(2:end,:); end
    tt=[tt;tk]; yy=[yy;yk]; vv=[vv;repmat(speeds(k),numel(tk),1)]; %#ok<AGROW>
end
rLocal=sqrt(max(yy(:,1),0)); rGlobal=d.epsilon*ones(size(tt));
sGlobal=(params.gamma*d.epsilon+params.noise)/params.rho*(1-exp(-params.rho*tt));
assert(all(rLocal<=rGlobal+1e-8) && all(yy(:,2)<=sGlobal+1e-8));
trajectory=table(tt,vv,rLocal,rGlobal,yy(:,2),sGlobal,'VariableNames',{'time','nominal_speed','local_r','original_epsilon','local_s','global_s'});
result.params=params; result.trajectory=trajectory;
result.processEnvelope=struct('Q',Q,'h',h,'c',c,'gGlobal',g1,'gBeta',gb,'conditionNumber',cond(Q));
result.residuals=struct('process',processResidual,'noise',noiseResidual,'dynamics',dynamicsResidual,'tracking',trackingResidual,'localSchur',localSchurResidual,'nonlinearExcess',maxExcess,'nonlinearSamples',accepted);
result.status='BASELINE_EQUIVALENT_DERIVATION; NUMERICAL_LMI_AUDIT; CONTINUOUS_DOMAIN_COVERAGE_UNPROVED';
result.outputDirectory=out;
save(fullfile(out,'lmi_radius_result.mat'),'result'); writetable(trajectory,fullfile(out,'lmi_radius_comparison.csv'));
fig=figure('Visible','off'); tiledlayout(3,1);
nexttile; plot(tt,vv); ylabel('Nominal speed'); grid on;
nexttile; plot(tt,rLocal,tt,rGlobal,'--'); ylabel('Estimation radius'); legend('Local forcing','Original offline epsilon'); grid on;
nexttile; plot(tt,yy(:,2),tt,sGlobal,'--'); ylabel('Tracking radius'); xlabel('Time (s)'); legend('Local forcing','Global forcing'); grid on;
exportgraphics(fig,fullfile(out,'lmi_radius_comparison.png'),'Resolution',160); close(fig);
fprintf('lambda=%g epsilon=%.10g global_q=%.10g recovered_epsilon=%.10g\n',params.lambda,d.epsilon,q0,sqrt(q0/params.lambda));
fprintf('Wxx condition=%g g1=%g gbeta=%g saving=%g q(beta)=%g\n',cond(Q),g1,gb,saving,q0-saving);
fprintf('LMI residuals: process=%g noise=%g dynamics=%g tracking=%g; saved tolerance=%g\n',processResidual,noiseResidual,dynamicsResidual,trackingResidual,tol);
fprintf('Local Schur residual=%g; nonlinear excess=%g (%d samples)\n',localSchurResidual,maxExcess,accepted);
fprintf('FINAL r_local=%.10g epsilon=%.10g improvement_percent=%g\n',rLocal(end),d.epsilon,100*(1-rLocal(end)/d.epsilon));
fprintf('FINAL s_local=%g s_global=%g\n',yy(end,2),sGlobal(end));
fprintf('BASELINE_RECOVERY_AND_LOCAL_ENVELOPE_TESTS_PASSED\nOUTPUT=%s\n',out);
end

function dy=lmi_rhs(y,speed,c)
r=sqrt(max(y(1),0)); s=max(y(2),0);
vupper=min(c.vmax,abs(speed)+c.cv*(r+s));
a=c.beta+(1-c.beta)*(vupper/c.vmax)^2;
q=c.q0-c.saving*(1-a)/(1-c.beta);
dy=[-c.lambda*y(1)+q; -c.rho*y(2)+c.gamma*r+c.noise];
end