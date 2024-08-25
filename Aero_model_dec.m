function [CX, CY, CZ, Cl, Cm, Cn, CT]=Aero_model_dec(alpha, beta, ele,ail, rud, dpf, dnf, ddf, qd, cbar,span,X_cg, X_cg_ref, Vt, p, q, r, Pa, Pa50, Tmax, Tidle, Tmil, dec) 
% Ref: Frederico R. Garza, Eugene A. Morelli, 
% A Collection of Nonlinear Aircraft Simulations in MATLAB,
% NASA/TM-2003-212145
%
% dec is for decoupling dec=1 for decoupling; 0 for none
dc1=1-dec;
% Aircraft Model is HL20
cx0a=[7.362e-02 -2.56e-04 -2.208e-04 -2.262e-06 2.996e-07 -3.64e-09 9.338e-12];
cx0b=[0 -5.299e-04 -4.709e-04 8.552e-05 -4.199e-06];
CX0=-polyval(fliplr(cx0a),alpha)-polyval(fliplr(cx0b),abs(beta))-1.295e-04*alpha*abs(beta);
cxde=[-1.854e-04 2.83e-06 -6.966e-07 1.323e-07 2.758e-09];
CXde=-polyval(fliplr(cxde),alpha);
cxda=[9.776e-04 -2.703e-05 -8.303e-06 6.645e-07 -1.273e-08];
CXda=-polyval(fliplr(cxda),alpha);
cxdr=[5.812e-04 1.41e-05 -2.585e-06 3.051e-07 8.161e-09];
CXdr=-polyval(fliplr(cxdr),alpha);
cyda=[3.357e-03 -1.661e-05 -3.28e-06 5.526e-8 -3.269e-10];
CYda=polyval(fliplr(cyda),alpha);
cydr=[1.855e-03 1.128e-05 6.069e-06 -1.78e-07 -1.886e-12];
CYdr=polyval(fliplr(cydr),alpha);
cyddf=[2.672e-05 -3.849e-05 4.564e-07 1.798e-08 -4.099e-10];
CYddf=polyval(fliplr(cyddf),alpha);
cz0a=[-9.025e-02 4.07e-02 3.094e-05 1.564e-05 -1.386e-06 2.545e-08 -1.189e-10];
cz0b=[0 2.564e-03 8.501e-04 -1.156e-04 3.416e-06];
CZ0=-polyval(fliplr(cz0a),alpha)-polyval(fliplr(cz0b),abs(beta))+4.862e-04*alpha*abs(beta);
czde=[5.140e-03 3.683e-05 -6.092e-06  2.818e-09   -2.459e-09];
CZde=-polyval(fliplr(czde),alpha);
cxdpf=[1.31e-04 1.565e-06 -1.542e-09];
CXdpf=-polyval(fliplr(cxdpf),alpha*alpha);
cxdnf=[-4.415e-04 -4.056e-06 -4.657e-07];
CXdnf=-polyval(fliplr(cxdnf),alpha);
cxddf=[-6.043e-04 -1.858e-05 8e-07 -4.848e-08 1.36e-09];
CXddf=-polyval(fliplr(cxddf),alpha);
CXdlg=CX0; %%%% ASSUMED!!!
czdpf=[3.779e-03 -7.017e-07 1.4e-10];
CZdpf=-polyval(fliplr(czdpf),alpha*alpha);
czdnf=[3.711e-03 -3.547e-05 -2.706e-06 2.938e-07 -5.552e-09];
CZdnf=-polyval(fliplr(czdnf),alpha);
CZdlg=CZ0; %%%% ASSUMED!!!
% Moment derivatives data
clda=[2.538e-03 1.963e-05 -3.725e-06 3.539e-08 -1.778e-10];
Clda=polyval(fliplr(clda), alpha);
cldr=[2.26e-04 -1.299e-05 5.565e-06 -3.382e-07 6.461e-09];
Cldr=polyval(fliplr(cldr), alpha);
clddf=[7.453e-04 -1.811e-05 -1.264e-07 9.972e-08 -2.684e-09];
Clddf=-polyval(fliplr(clddf), alpha);
clp=[-0.5261357  0.029749438  -0.006960559  0.00053564  -1.35043e-05];
Clp=polyval(fliplr(clp), alpha);
clr=[0.497793859 -0.028742272  0.016480243  -0.001439044  3.56179e-05];
Clr=polyval(fliplr(clr), alpha);
cm0a=[2.632e-02 -2.226e-03 -1.85894e-05 6.001e-07 1.828e-07 -9.733e-09 1.71e-10];
cm0b=[0 -5.233e-04 6.795e-05 -1.993e-05 1.341e-06]; 
Cm0=polyval(fliplr(cm0a),alpha)-polyval(fliplr(cm0b),abs(beta))+6.061e-05*alpha*abs(beta);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cn0=2.28e-03*beta+1.79e-06*beta^3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmde=[-1.903e-03 -1.593e-05 2.611e-06 5.116e-08 1.626e-09];
Cmde=polyval(fliplr(cmde),alpha);
Cmdlg=Cm0; %%%% ASSUMED!!!
cmdpf=[-9.896e-04 -1.494e-09 6.303e-11];
Cmdpf=polyval(fliplr(cmdpf),alpha*alpha);
cmdnf=[-1.086e-03 1.57e-05 4.174e-07  -1.133e-07 2.723e-09];
Cmdnf=polyval(fliplr(cmdnf),alpha);
cmq=[-0.195552417 -0.006776550  0.00285193  -0.000184146   2.45653e-06];
Cmq=polyval(fliplr(cmq),alpha);
cnda=[-2.769e-03 -4.377e-05  9.952e-06  -3.642e-07 4.692e-09];
Cnda=polyval(fliplr(cnda),alpha);
cndr=[-1.278e-03 1.32e-05 -4.72e-06  2.371e-07 -3.340e-09];
Cndr=polyval(fliplr(cndr),alpha);
cnddf=[-5.107e-05 1.108e-05  -1.547e-08  -1.552e-08 1.413e-10];
Cnddf=polyval(fliplr(cnddf),alpha);
cnp=[0.385147809  -0.001356061  -0.00209489  8.52445e-05 1.02309e-07]; 
Cnp=polyval(fliplr(cnp),alpha);
cnr=[-0.790855552 -0.010770449 -0.00095452  0.000151569  -5.13541e-06];
Cnr=polyval(fliplr(cnr),alpha);
%{
CX = CX0+CXde*ele+CXda*abs(ail)+CXdr*abs(rud)+CXdpf*dpf+CXdnf*dnf+CXddf*abs(ddf)+(CXdlg-CX0);
CY= -0.01242*beta+CYda*ail+CYdr*rud+CYddf*ddf;
CZ= CZ0+CZde*ele+CZdpf*dpf+CZdnf*dnf+(CZdlg-CZ0);
Cl=-0.00787*beta+Clda*ail+Cldr*rud+Clddf*ddf+(span/(2*Vt))*(Clp*p+Clr*r); % rolling moment coeff.
Cm=Cm0+Cmde*ele+Cmdpf*dpf+Cmdnf*dnf+(Cmdlg-Cm0)+(q*cbar/(2*Vt))*Cmq+(X_cg_ref-X_cg)*CZ; % pitching moment coeff.
Cn=Cn0+Cnda*ail+Cndr*rud+Cnddf*ddf+(span/(2*Vt))*(Cnp*p+Cnr*r)-(cbar/span)*(X_cg_ref-X_cg)*CY; % yawing moment coeff.
%}

CXn=CXdlg;CZn=CZdlg;CYls=-0.01242*beta;
Cln=-0.00787*beta; % rolling moment coeff, nonlinear, other
Cmn=Cmdlg+(X_cg_ref-X_cg)*CZn; % pitching moment coeff. nonlinear
Cnn=Cn0-(cbar/span)*(X_cg_ref-X_cg)*CYls; % yawing moment coeff., nonlinear other
% Use Elevator, aileron & rudder to cancel, Cln;Cmn;Cnn in attitude
% dynamics
amat=[0 Clda Cldr;Cmde 0 0 ;0 Cnda Cndr]+[0 0 0;(X_cg_ref-X_cg)*[CZde 0 0];-(cbar/span)*(X_cg_ref-X_cg)*[0 CYda CYdr]];
ccl=amat\[Cln;Cmn;Cnn];
% update Elevator, aileron & rudder delections
ele1=ele+ccl(1); ail1=ail+ccl(2);rud1=rud+ccl(3);
CXlc=[CXde CXda CXdr CXdpf CXdnf CXddf  0]*[ele1 abs(ail1) abs(rud1) dpf dnf abs(ddf) 0]';
CX=CXn+CXlc;

CYlc=[0 CYda CYdr 0 0 CYddf 0]*[ele1 ail1 rud1 dpf dnf ddf 0]';
CY=CYls+CYlc;

CZlc=[CZde 0 0 CZdpf CZdnf 0 0]*[ele1 ail1 rud1 dpf dnf ddf 0]';
CZ=CZn+CZlc;

Clls=(span/(2*Vt))*(Clp*p+Clr*r); % rolling moment coeff, linear attitude state
Cllc=[0 Clda Cldr 0 0 Clddf 0]*[ele ail rud dpf dnf ddf 0]'; % rolling moment coeff. linear control terms
Cl=dc1*Cln+Cllc+Clls;

Cmls=(q*cbar/(2*Vt))*Cmq; % pitching moment coeff. linear attitude state dependent
Cmlc=[Cmde 0 0 Cmdpf Cmdnf 0 0]*[ele ail rud dpf dnf ddf 0]'; % pitching moment coeff.
Cm=dc1*Cmn+Cmls+Cmlc;

Cnls=(span/(2*Vt))*(Cnp*p+Cnr*r); % yawing moment coeff.
Cnlc=[0 Cnda Cndr 0 0 Cnddf 0]*[ele ail rud dpf dnf ddf 0]'; % yawing moment coeff.
Cn=dc1*Cnn+Cnls+Cnlc;
% Engine Model based on F16
%TH =0; %Not defined yet! Defined below; Pa is actuual Engine power
if Pa<Pa50
    TH= Tidle+(Tmil-Tidle)*(Pa/Pa50);
else
    TH=Tmil+(Tmax-Tmil)*((Pa-Pa50)/Pa50);
end
CT = TH/qd;

