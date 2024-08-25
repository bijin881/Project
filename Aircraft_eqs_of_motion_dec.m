function xdot=Aircraft_eqs_of_motion_dec(t,x, parm, controls,demands, dec,icon)
% Aircraft Equations of Motion
% Ref: Frederico R. Garza, Eugene A. Morelli, 
% A Collection of Nonlinear Aircraft Simulations in MATLAB,
% NASA/TM-2003-212145
%
% Conversion factors to SI Units
% ft=0.3048; pas=47.8803;
% Conversion factors:  1 Btu = 778.3 ft-lbf; 1 hp=2544.4335776441 Btu/hr;
%  1 hp=746 watts; 1 BTU=1055.06 Joules/ 1 watt=1 Joule/sec;
% 1 BTU/hr= 0.000277778 BTU/s
% F16 Max Thrust 23770 lbf; Mil: 60% of max; idle: 20% of max;
% Engines: (single engine)
% Pratt & Whitney F100-PW-200 afterburning turbofan, a modified version of 
% the F-15's F100-PW-100, rated at 23,830 lbf (106.0 kN) thrust. 
% Later replaced by the standard F-16 engine through the Block 25, 
% except for the newly built Block 15s with the 
% Operational Capability Upgrade (OCU). 
% The OCU introduced the 23,770 lbf (105.7 kN) F100-PW-220
% SI=1 % SI Units
pi=atan(1)*4; piby6=pi/6;% pi=3.141592653589793;
rads=pi/180;
SI=0; % American Units
g=9.81;
if SI==0 % American Units
    g=32.174;
end
% INPUT Parameter
% F4 Model; Please USE HL20 model
% W=38924; Ixx=24970;Iyy=122190;Izz=139800; Ixz=1175; mass=W/g;
% For HL 20 Model
% W=19100; Ixx=7512; Iyy=35594; Izz=35644; Ixz=0; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHECK!
% S=300; %CHECK (ASSUMED!) in ft^2; US units.
% cbar=15; span=28; X_cg=0.54*cbar;X_cg_ref=0.54*cbar; % ALL ASSUMED ; Could be incorrect
% Ieng=1000; weng=100; % Tmax/W=1.25;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: HL20 has SEVEN controls: ele,ail, rud, dpf, dnf, ddf, and delth
% Extract parameters, controls and demands
W=parm(1); Ixx=parm(2); Iyy=parm(3); Izz= parm(4); Ixz=parm(5); S=parm(6); cbar=parm(7); 
span=parm(8); X_cg=parm(9); X_cg_ref=parm(10); Ieng=parm(11); weng=parm(12); Tmax=parm(13); Tmil=parm(14); Tidle=parm(15);
%controls(1)=ele;controls(2)=ail; controls(3)=rud; controls(4)=dpf; controls(5)=dnf; controls(6)=ddf; controls(7)=delth;
ele=controls(1);ail=controls(2); rud=controls(3); dpf=controls(4); dnf=controls(5); ddf=controls(6); delth=controls(7);
hd=demands(1);ud=demands(2);

% Engine model Data (Based on F16)
% Commanded Power Pc
%delth=0.7; % throttle setting 0<=delth<=1
Pa=x(13);
Pa50=50; 
if delth>0
if delth<0.77
    Pc=64.94*delth;
else
    Pc=217.38*delth-117.38; % Pc=0 for delth=0.53997607875;
end
if (Pc-Pa)>=Pa50/2
    itoustar=1;
elseif (Pc-Pa)>=Pa50
    itoustar=0.1;
else
    itoustar=1.9-0.036*(Pc-Pa);
end
if Pc>Pa50
    if Pa> Pa50
        Pca=Pc;
        itoue=5;
    else
        Pca=1.2*Pa50;
        itoue=itoustar;
    end
else
    if Pa50>Pa50
        Pca=0.8*Pa50;
        itoue=5;
    else
        Pca=Pc;
        itoue=itoustar;
    end
end
else
    Pca=0;itoue=5;
end
% Engine static parameters model Complete
mass=W/g;
Idel=Ixx*Izz-Ixz*Ixz;
c1=((Iyy-Izz)*Izz-Ixz*Ixz)/Idel; c2=((Ixx-Iyy+Izz)*Ixz)/Idel; c3=Izz/Idel;
c4=Ixz/Idel; c5=(Izz-Ixx)/Iyy; c6= Ixz/Iyy; c7=1/Iyy; c8= ((Ixx-Iyy)*Ixx-Ixz*Ixz)/Idel; c9=Ixx/Idel;
% Rename states
u=x(1);v=x(2);w=x(3);p=x(4);q=x(5);r=x(6);phi=x(7);theta=x(8);psi=x(9);xe=x(10);ye=x(11);h=x(12);Pa=x(13); ele_in=x(14);
Vt=sqrt(u*u+v*v+w*w);alpha=atan2(w, u);beta=asin(v/Vt); % CONVERT Angles to Degs????
gamma=theta-alpha;
alpha=alpha/rads;beta=beta/rads; % in degrees!
[qd, Mach]=atmos(h,Vt, SI);
qds=qd*S/mass; heng=Ieng*weng;
if icon==1
% LQR Synthesis with Linearised Model
% Needs Stability augmentation Control laws here
%
%
% apply feedbacks
Q=diag([1 1 1 100 100 100 10 10 10 1]);R=0.1*eye(3);

  [a,b, ext, K]=Linear_model(parm,qds, Vt, Pa, Pa50, Q,R) ; %Linear model with LQR
  %size(b)
  %size(K)
  %size(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Controls
%%%%%%%%%%% Changes here  %%%%%%%%%%%%%%%%%%%%%%%%%
xd=[ud 0 0 0 0 0 0 0 0 0 0 hd 0 0]';
% K contains LQR control law
% Controls synthesised in degrees!
gm=1;% gm was 2; reset to 1
K(2,4)=K(2,4)-2*0;% add negative feedback if necessary (remove *0)
u3=-gm*K*(x-xd); % Full state feedback from LQR
ail=ail+u3(2);rud=rud+u3(3);ele_in0=u3(1);
% Do we reall need this; Cut it out and see%
% ele_in0=ele_in0-0.2*(h-hd)+0.1*(w+2*(u-ud)); % additional feedbacks!

controls=[ele_in0 ail rud dpf dnf ddf delth];
end
% Apply limits & convert to degrees, if needed! (not needed here!)
  controls=Limiters(controls,0);
  ele=controls(1);ail=controls(2); rud=controls(3); dpf=controls(4); dnf=controls(5); ddf=controls(6); 
% Control dynamic state
ele_in_dot=-10*(ele_in-ele); % Feedback state: defines control law
% Controls definition Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[CX, CY, CZ, Cl, Cm, Cn, CT]=Aero_model_dec(alpha, beta, ele_in,ail, rud, dpf, dnf, ddf, qd, cbar,span,X_cg, X_cg_ref, Vt, p, q, r, Pa, Pa50, Tmax, Tidle, Tmil, dec);  % defined!
if icon==2
    ku=0.1;kw=0.1; kh=0.1; ud=60;wd=0; hd=10000; CYv=-0.1;
    CW=W/(qd*S);
    Vt=sqrt(u*u+v*v+w*w);% alpha=atan2(w, u);beta=asin(v/Vt);
    sa=w/sqrt(w*w+u*u); ca=u/sqrt(w*w+u*u);sb=v/Vt; cb=cos(asin(v/Vt));st=sin(theta); ct=cos(theta);
    Tc=cb*[ca sa]*[(st-ku*(u-ud))*CW-CX;-CW*(ct+kw*(w-wd)-kh*(h-hd))-CZ]-(CY+CYv*v)*sb;
    Lc=[-sa ca]*[(st-ku*(u-ud))*CW-CX;-CW*(ct+kw*(w-wd)-kh*(h-hd))-CZ];
    CX=CX+Tc*ca*cb-Lc*sa;CY=CY+Tc*sb;CZ=CZ+Tc*sa*cb+Lc*ca;
end
udot=r*v-q*w-g*sin(theta)+qds*(CX+CT);
vdot=p*w-r*u+g*cos(theta)*sin(phi)+qds*CY;
wdot=q*u-p*v+g*cos(theta)*cos(phi)+qds*CZ;
pdot=(c1*r+c2*p+c4*heng)*q+qd*S*span*(c3*Cl+c4*Cn);
qdot=(c5*p-c7*heng)*r-c6*(p*p-r*r)+qd*S*cbar*c7*Cm;
rdot=(c8*p-c2*r+c9*heng)*q+qd*S*span*(c4*Cl+c9*Cn);
phi_dot=p+tan(theta)*(q*sin(phi)+r*cos(phi));
theta_dot=q*cos(phi)-r*sin(phi);
psi_dot=(q*sin(phi)+r*cos(phi))/cos(theta);
xe_dot=u*cos(psi)*cos(theta)+v*(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))+w*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
ye_dot=u*sin(psi)*cos(theta)+v*(sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi))+w*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi));
hdot=u*sin(theta)-v*cos(theta)*sin(phi)-w*cos(theta)*cos(phi); % height rate
% Engine Dynamic State
Padot=itoue*(Pca-Pa); % Engine Model State
% Dynamics Definition Complete
% Assemple Equations
  xdot=[udot;vdot;wdot; pdot; qdot; rdot; phi_dot; theta_dot;psi_dot;xe_dot;ye_dot;hdot;Padot; ele_in_dot];
% tranform states
%[Vt, alpha, beta, Vtdot, alpha_dot, beta_dot]=state_trans(u, v, w, udot, vdot, wdot);
% Note: u=Vt*cos(alpa)*cos(beta); v=Vt*sin(beta); w=Vt*sin(alpa)*cos(beta);
