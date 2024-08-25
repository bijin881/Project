% HL20.m
clear all 
close all
clc
pi=atan(1)*4; piby6=pi/6;% pi=3.141592653589793;
rads=pi/180;
SI=0; % American Units
g=9.81;
if SI==0 % American Units
    g=32.174;
end
% For HL 20 Model: Define properties, parameters
W=19100; Ixx=7512; Iyy=35594; Izz=35644; Ixz=0; mass=W/g;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CHECK!
% span=13.89; S=286.45; Chord=28.24;
S=286.45; %CHECK (ASSUMED!) in ft^2; US units.
cbar=28.24; span=13.89; X_cg=0.54*cbar;X_cg_ref=0.54*cbar; % ALL ASSUMED ; Could be incorrect
Ieng=1000; weng=100; % Tmax/W=1.25;
Tmax=0.01*W; Tmil=0.6*Tmax; Tidle=0.2*Tmax;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: HL20 has SEVEN controls: ele,ail, rud, dpf, dnf, ddf, and delth
dist=0.2;
parm=[W Ixx Iyy Izz Ixz S cbar span X_cg X_cg_ref Ieng weng  Tmax Tmil Tidle dist];
disp('HL20 Control Synthesis & Simulation')
disp('Display Parameters size')
size(parm)
% define Controls
% Note: HL20 has SEVEN controls: ele,ail, rud, dpf, dnf, ddf, and delth
delth=0.001; % Was delth=0.001;
ele=0.0*rads; ail=0; rud=0; dpf=0.0*rads*0; dnf=-0.0*rads*0; ddf=0*rads; % Controls mostly in rads
controls=[ele ail rud dpf dnf ddf delth]
nfl=6;
% parm(1)=W; parm(2)=Ixx; parm(3)=Iyy; parm(4)=Izz; parm(5)=Ixz; parm(6)=S; 
% parm(7)=cbar;parm(8)=X_cg; parm(9)= X_cg_ref;parm(10)=Ieng;parm(11)=weng;

% define initial conditions for states
% Rename states
% u=x(1);v=x(2);w=x(3);p=x(4);q=x(5);r=x(6);phi=x(7);theta=x(8);psi=x(9);
% xe=x(10);ye=x(11);h=x(12);Pa=x(13);
hd=10000; ud=60; 
% initialize states!
yi=zeros(1,14); % NOT Correct MUST re-define
yi=[ud 0 0 0 0 0 0 0 0 0 0 hd 0 0];
yi=yi';
%size(yi)
%
s=yi;
xV=[];
time=[];
N=2000;
t0=0;dt=0.001;
t=t0;
dec=1; % decoupling: dec is for decoupling dec=1 for decoupling;0 for none 
icon=2;
for k=1:N
     kstar=k-1000*floor(k/1000) ;
  if kstar==0
      k
  end
  % if k>20
  %    controls(7)=0;
  % end
  % Apply feedbacks
  
  % elevator
  %ele=ele+s(14);
  %controls(1)=ele;

  % Apply limits & convert to degrees
  controls=Limiters(controls, 1);
  %%%%%%
  demands=[hd ud];
  % Simulate process and measurement
  tspan=[(k-1)*dt k*dt];
  options = odeset('RelTol',1e-6,'AbsTol',1e-7);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  [tfin,yfin]=ode45(@(t,y)Aircraft_eqs_of_motion_dec(t,y,parm, controls,demands, dec, icon),tspan,s);
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  [nt nj]=size(yfin);
  s0=yfin(nt,:)';
  s = s0 ;% update process for the next step
  % Accumulate results
  kstar=k-10*floor(k/10) ;
  if kstar==0
  time=[time t];
  xV=[xV s];
  end
  t=t+dt;
end
%size(xV);
figure(1)
for k=1:6                                 % plot results
  subplot(6,1,k)
  
if k==1
    plot(time, xV(1,:), '-m', 'lineWidth', 3)
    title('HL20 Response plots')
    ylabel('Fwd u')
  elseif k==2
       plot(time, xV(3,:), '-r', 'lineWidth', 3)
      ylabel('Vert. w')
  elseif k==3
      plot(time,xV(4,:), '-b', 'lineWidth', 3)
      ylabel('Roll Rate')
  elseif k==4
      plot(time, xV(5,:), '-b', 'lineWidth', 3)
      ylabel('Pitch Rate')
  elseif k==5
      plot(time, xV(6,:), '-b', 'lineWidth', 3)
      ylabel('Yaw Rate')    
  elseif k==6
      plot(time, xV(12,:), '-m', 'lineWidth', 3)
      ylabel('Height')
end
xlabel('Time in seconds')
grid on
end
