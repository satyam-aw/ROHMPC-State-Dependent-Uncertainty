function [dy,scale,speedUpper] = bound_rhs(~,y,nominalSpeed,c)
% Comparison dynamics, conditional on true states remaining in the design domain.
% r bounds ||x-xhat||_P; s bounds ||xhat-z||_P. Both affect true speed.
r=max(y(1),0); s=max(y(2),0);
speedUpper=min(c.maximumSpeed,abs(nominalSpeed)+c.velocityMetricGain*(r+s));
scale=c.beta+(1-c.beta)*(speedUpper/c.maximumSpeed)^2;
forcing=c.processOther+c.processAcceleration*scale+c.noise;
dy=[-c.rhoObserver*y(1)+forcing; -c.rhoController*y(2)+c.gamma*r+c.noise];
end