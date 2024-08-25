function [Vt, alpha, beta, Vtdot, alpha_dot, beta_dot]=state_trans(u, v, w, udot, vdot, wdot)
Vt=sqrt(u*u+v*v+w*w);
alpha=atan2(w, u);
beta=asin(v/Vt);
Vtdot=(u*udot+v*vdot+w*wdot)/Vt;
uw2=u*u+w*w; uw=sqrt(uw2);
alpha_dot=(u*wdot-w*udot)/(u*u+w*w);
beta_dot=(Vt*vdot-v*Vt_dot)/(Vt*uw);
% trans formation  to solve for [udot;vdot;wdot]
% from [Vt_dot; alpha_dot; beta_dot]=
%[u/Vt v/Vt w/Vt;-w/uw2 0 u/uw2; -v*u/Vt (uw2/Vt) -v*w/Vt]*[udot;vdot;wdot]
uc=Vt*cos(alpa)*cos(beta); vc=Vt*sin(beta); wc=Vt*sin(alpa)*cos(beta);
tol=1.0e-06;
if abs(uc-u)+abs(vc-v)+abs(wc-w)>tol
    disp('Error in Transformation')
end