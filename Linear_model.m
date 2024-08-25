function [a,b, ext, K]=Linear_model(parm,qds, Vt, Pa, Pa50, Q,R) 
% Ref: Frederico R. Garza, Eugene A. Morelli, 
% A Collection of Nonlinear Aircraft Simulations in MATLAB,
% NASA/TM-2003-212145
%
g=32.174;
W=parm(1); Ixx=parm(2); Iyy=parm(3); Izz= parm(4); Ixz=parm(5); S=parm(6); cbar=parm(7); 
span=parm(8); X_cg=parm(9); X_cg_ref=parm(10); Ieng=parm(11); weng=parm(12); Tmax=parm(13); Tmil=parm(14); Tidle=parm(15); dist=parm(16);
Idel=Ixx*Izz-Ixz*Ixz;
c1=((Iyy-Izz)*Izz-Ixz*Ixz)/Idel; c2=((Ixx-Iyy+Izz)*Ixz)/Idel; c3=Izz/Idel;
c4=Ixz/Idel; c5=(Izz-Ixx)/Iyy; c6= Ixz/Iyy; c7=1/Iyy; c8= ((Ixx-Iyy)*Ixx-Ixz*Ixz)/Idel; c9=Ixx/Idel;
mass=W/g; dXcg=X_cg_ref-X_cg;heng=Ieng*weng;
% Aircraft Model is HL20
CYde=0;  CYdpf=0; CYdnf=0;  CZda=0; CZdr=0; CZddf=0;
Clde=0;  Cldpf=0; Cldnf=0;  Cmda=0; Cmdr=0;  Cmddf=0;Cnde=0;  Cndpf=0; Cndnf=0; 
cx0a=[7.362e-02 -2.56e-04 -2.208e-04 -2.262e-06 2.996e-07 -3.64e-09 9.338e-12];
cx0b=[0 -5.299e-04 -4.709e-04 8.552e-05 -4.199e-06];
%CX0=-polyval(fliplr(cx0a),alpha)-polyval(fliplr(cx0b),abs(beta))-1.295e-04*alpha*abs(beta);
CX00=-cx0a(1); CX0a=-cx0a(2); CX0b=-cx0b(2);
cxde=[-1.854e-04 2.83e-06 -6.966e-07 1.323e-07 2.758e-09];
%CXde=-polyval(fliplr(cxde),alpha);
CXde=-cxde(1);
cxda=[9.776e-04 -2.703e-05 -8.303e-06 6.645e-07 -1.273e-08];
%CXda=-polyval(fliplr(cxda),alpha);
CXda=-cxda(1);
cxdr=[5.812e-04 1.41e-05 -2.585e-06 3.051e-07 8.161e-09];
%CXdr=-polyval(fliplr(cxdr),alpha);
CXdr=-cxdr(1);
cyda=[3.357e-03 -1.661e-05 -3.28e-06 5.526e-8 -3.269e-10];
CYda=cyda(1);
%CYda=polyval(fliplr(cyda),alpha);
cydr=[1.855e-03 1.128e-05 6.069e-06 -1.78e-07 -1.886e-12];
CYdr=cydr(1);
%CYdr=polyval(fliplr(cydr),alpha);
cyddf=[2.672e-05 -3.849e-05 4.564e-07 1.798e-08 -4.099e-10];
CYddf=cyddf(1);
%CYddf=polyval(fliplr(cyddf),alpha);
cz0a=[-9.025e-02 4.07e-02 3.094e-05 1.564e-05 -1.386e-06 2.545e-08 -1.189e-10];
cz0b=[0 2.564e-03 8.501e-04 -1.156e-04 3.416e-06];
%CZ0=-polyval(fliplr(cz0a),alpha)-polyval(fliplr(cz0b),abs(beta))+4.862e-04*alpha*abs(beta);
CZ00=-cz0a(1);CZ0a=-cz0a(2);CZ0b=-cz0b(2);
czde=[5.140e-03 3.683e-05 -6.092e-06  2.818e-09   -2.459e-09];
%CZde=-polyval(fliplr(czde),alpha);
CZde=-czde(1);
cxdpf=[1.31e-04 1.565e-06 -1.542e-09];
%CXdpf=-polyval(fliplr(cxdpf),alpha*alpha);
CXdpf=-cxdpf(1);
cxdnf=[-4.415e-04 -4.056e-06 -4.657e-07];
%CXdnf=-polyval(fliplr(cxdnf),alpha);
CXdnf=-cxdnf(1);
cxddf=[-6.043e-04 -1.858e-05 8e-07 -4.848e-08 1.36e-09];
%CXddf=-polyval(fliplr(cxddf),alpha);
CXddf=-cxddf(1);
%CXdlg=CX0; %%%% ASSUMED!!!
czdpf=[3.779e-03 -7.017e-07 1.4e-10];
%CZdpf=-polyval(fliplr(czdpf),alpha*alpha);
CZdpf=-czdpf(1);
czdnf=[3.711e-03 -3.547e-05 -2.706e-06 2.938e-07 -5.552e-09];
%CZdnf=-polyval(fliplr(czdnf),alpha);
CZdnf=-czdnf(1);
%CZdlg=CZ0; %%%% ASSUMED!!!
% Moment derivatives data
clda=[2.538e-03 1.963e-05 -3.725e-06 3.539e-08 -1.778e-10];
%Clda=polyval(fliplr(clda), alpha);
Clda=clda(1);
cldr=[2.26e-04 -1.299e-05 5.565e-06 -3.382e-07 6.461e-09];
%Cldr=polyval(fliplr(cldr), alpha);
Cldr=cldr(1);
clddf=[7.453e-04 -1.811e-05 -1.264e-07 9.972e-08 -2.684e-09];
%Clddf=-polyval(fliplr(clddf), alpha);
Clddf=-clddf(1);
clp=[-0.5261357  0.029749438  -0.006960559  0.00053564  -1.35043e-05];
%Clp=polyval(fliplr(clp), alpha);
Clp=clp(1);
clr=[0.497793859 -0.028742272  0.016480243  -0.001439044  3.56179e-05];
%Clr=polyval(fliplr(clr), alpha);
Clr=clr(1);
cm0a=[2.632e-02 -2.226e-03 -1.85894e-05 6.001e-07 1.828e-07 -9.733e-09 1.71e-10];
cm0b=[0 -5.233e-04 6.795e-05 -1.993e-05 1.341e-06]; 
%Cm0=polyval(fliplr(cm0a),alpha)-polyval(fliplr(cm0b),abs(beta))+6.061e-05*alpha*abs(beta);
Cm00=cm0a(1);Cm0a=cm0a(2); Cm0b=cm0b(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Cn0=2.28e-03*beta+1.79e-06*beta^3;
Cn0b=2.28e-03;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cmde=[-1.903e-03 -1.593e-05 2.611e-06 5.116e-08 1.626e-09];
%Cmde=polyval(fliplr(cmde),alpha);
Cmde=cmde(1);
%Cmdlg=Cm0; %%%% ASSUMED!!!
cmdpf=[-9.896e-04 -1.494e-09 6.303e-11];
%Cmdpf=polyval(fliplr(cmdpf),alpha*alpha);
Cmdpf=cmdpf(1);
cmdnf=[-1.086e-03 1.57e-05 4.174e-07  -1.133e-07 2.723e-09];
%Cmdnf=polyval(fliplr(cmdnf),alpha);
Cmdnf=cmdnf(1);
cmq=[-0.195552417 -0.006776550  0.00285193  -0.000184146   2.45653e-06];
%Cmq=polyval(fliplr(cmq),alpha);
Cmq=cmq(1);
cnda=[-2.769e-03 -4.377e-05  9.952e-06  -3.642e-07 4.692e-09];
%Cnda=polyval(fliplr(cnda),alpha);
Cnda=cnda(1);
cndr=[-1.278e-03 1.32e-05 -4.72e-06  2.371e-07 -3.340e-09];
%Cndr=polyval(fliplr(cndr),alpha);
Cndr=cndr(1);
cnddf=[-5.107e-05 1.108e-05  -1.547e-08  -1.552e-08 1.413e-10];
%Cnddf=polyval(fliplr(cnddf),alpha);
Cnddf=cnddf(1);
cnp=[0.385147809  -0.001356061  -0.00209489  8.52445e-05 1.02309e-07]; 
%Cnp=polyval(fliplr(cnp),alpha);
Cnp=cnp(1);
cnr=[-0.790855552 -0.010770449 -0.00095452  0.000151569  -5.13541e-06];
%Cnr=polyval(fliplr(cnr),alpha);
Cnr=cnr(1);
Clb=-0.00787;
%CX = CX0+CXde*ele+CXda*abs(ail)+CXdr*abs(rud)+CXdpf*dpf+CXdnf*dnf+CXddf*abs(ddf)+(CXdlg-CX0);
%CY= -0.01242*beta+CYda*ail+CYdr*rud+CYddf*ddf;
%CZ= CZ0+CZde*ele+CZdpf*dpf+CZdnf*dnf+(CZdlg-CZ0);
%Cl=-0.00787*beta+Clda*ail+Cldr*rud+Clddf*ddf+(span/(2*Vt))*(Clp*p+Clr*r); % rolling moment coeff.
%Cm=Cm0+Cmde*ele+Cmdpf*dpf+Cmdnf*dnf+(Cmdlg-Cm0)+(q*cbar/(2*Vt))*Cmq+(X_cg_ref-X_cg)*CZ; % pitching moment coeff.
%Cn=Cn0+Cnda*ail+Cndr*rud+Cnddf*ddf+(span/(2*Vt))*(Cnp*p+Cnr*r)-(cbar/span)*(X_cg_ref-X_cg)*CY; % yawing moment coeff.
%Define matrices
a12=Vt*[0 0 0; 0 0 -1;0 1 0]; a13=g*[0 -1 0; 1 0 0; 0 0 0 ];a11=(qds/Vt)*[ 0 CX0b CX0a;0 -0.01242 0; 0 CZ0b CZ0a];
a220=heng*[0 c4 0;0  0 -c7; 0 c9 0];a221= (0.5/Vt)*[span*Clp 0 span*Clr;0 cbar*Cmq 0;span*Cnp 0 span*Cnr]; a22=a220+a221;
a210=mass*qds*[span*c3 0 span*c4; 0 cbar*c7 0; span*c4 0 span*c9]*[0 Clb 0;0 Cm0b Cm0a; 0 Cn0b 0];
a211=dXcg*[0 0 0; 0 CZ0b CZ0a; 0  0.01242*cbar/span  0]; a21=a210+a211;

% Engine Model based on F16
%TH =0; %Not defined yet! Defined below; Pa is actuual Engine power
if Pa<Pa50
    TH= Tidle+(Tmil-Tidle)*(Pa/Pa50);
else
    TH=Tmil+(Tmax-Tmil)*((Pa-Pa50)/Pa50);
end
CT = S*TH/(qds*mass);
ext1=qds*[CX00+CT;0;CZ00]+[0;0;g]; ext2=mass*qds*[span*c3 0 span*c4; 0 cbar*c7 0; span*c4 0 span*c9]*[ 0;Cm00; 0]+dXcg*[0; CZ00;0];
CXYZ=qds*[CXde CXda CXdr CXdpf CXdnf CXddf;CYde CYda CYdr CYdpf CYdnf CYddf;CZde CZda CZdr CZdpf CZdnf CZddf];
Clmn=[Clde Clda Cldr Cldpf Cldnf Clddf;Cmde Cmda Cmdr Cmdpf Cmdnf Cmddf;Cnde Cnda Cndr Cndpf Cndnf Cnddf];
Clmn=mass*qds*[span*c3 0 span*c4; 0 cbar*c7 0; span*c4 0 span*c9]*Clmn+dXcg*[0*CXYZ(1,:);CXYZ(3,:);-CXYZ(2,:)];
CXYZ=qds*CXYZ;
% Final model & LQR
a=[a11 a12 a13 [0;0;0]; a21 a22 0*eye(3) [0;0;0]; 0*eye(3) eye(3) 0*eye(3) [0;0;0]; [0 0 -1 0 0 0 0 Vt 0 0]];
b=[CXYZ(:,1:3); Clmn(:, 1:3);0*eye(3); [0 0 0]];
ext=[ext1;ext2; [0;0;0]; 0];
% disp('Number of states in linear Model=10')
% The 10 states are:
% u=x(1);v=x(2);w=x(3);p=x(4);q=x(5);r=x(6);phi=x(7);theta=x(8);psi=x(9);h=x(12);
% So inflate the control  law to full state model in line 151
aa=a+dist*eye(10);
Ko=lqr(aa, b, Q, R); % Controls are synthesised in degrees!
z3=[0;0;0];
K=[Ko(1:3,1:9) z3 z3 Ko(1:3,10) z3 z3]; % this feedback law is for all 14 states
    
