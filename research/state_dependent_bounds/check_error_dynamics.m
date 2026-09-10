function report = check_error_dynamics(model,cert,seed)
% Numerical falsification test of the comparison inequalities; not a proof.
old=rng; cleanup=onCleanup(@() rng(old)); rng(seed,'twister'); %#ok<NASGU>
[f,~,~]=model.get_fABfun(); R=cert.R; P=cert.P;
maxObs=-inf; maxTrack=-inf; accepted=0;
for k=1:1500
    z=model.x_lb+(model.x_ub-model.x_lb).*(0.25+0.5*rand(model.nx,1));
    vn=model.u_lb+(model.u_ub-model.u_lb).*(0.25+0.5*rand(model.nu,1));
    a=randn(model.nx,1); b=randn(model.nx,1);
    e=R\((1e-4+0.003*rand)*a/norm(a));
    delta=R\((1e-4+0.003*rand)*b/norm(b));
    xhat=z+delta; x=xhat+e; u=vn+cert.K*delta;
    if any(x<model.x_lb | x>model.x_ub) || any(xhat<model.x_lb | xhat>model.x_ub) || any(u<model.u_lb | u>model.u_ub)
        continue;
    end
    w=model.w_min+(model.w_max-model.w_min).*double(rand(model.nw,1)>0.5);
    scale=cert.beta+(1-cert.beta)*(norm(x(7:9))/cert.maximumSpeed)^2;
    w(cert.scaledChannels)=scale*w(cert.scaledChannels);
    eta=model.eta_min+(model.eta_max-model.eta_min).*double(rand(model.neta,1)>0.5);
    edot=full(f(x,u))-full(f(xhat,u))-cert.L*model.C*e+model.E*w-cert.L*model.F*eta;
    ddot=full(f(xhat,u))-full(f(z,vn))+cert.L*model.C*e+cert.L*model.F*eta;
    r=norm(R*e); s=norm(R*delta);
    comparison=bound_rhs(0,[r;s],norm(z(7:9)),cert);
    maxObs=max(maxObs,(e'*P*edot)/r-comparison(1));
    maxTrack=max(maxTrack,(delta'*P*ddot)/s-comparison(2));
    accepted=accepted+1;
end
assert(accepted>=1000,'Too few admissible nonlinear error samples.');
assert(maxObs<=1e-9 && maxTrack<=1e-9,'A nonlinear error derivative exceeded its proposed bound.');
report.samples=accepted; report.maxObserverExcess=maxObs; report.maxTrackingExcess=maxTrack;
fprintf('Nonlinear error audit: %d samples, max observer excess=%g, tracking excess=%g\n',accepted,maxObs,maxTrack);
end