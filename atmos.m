function [q, Mach]=atmos(h,Vt,SI)
% Input in metres and metre/s
% Conversion factors to SI Units
if SI==1
ft=0.3048;
hft=h/ft;
vtfps=Vt/ft;
else
    hft=h;
    vtfps=Vt;
end
Tstar=1-0.703e-05*hft;
rho=0.002377*Tstar^4.14;
if hft<35000
    Mach=vtfps/sqrt(1.4*1716.3*519*Tstar);
else
    Mach=vtfps/sqrt(1.4*1716.3*390);
end
qpsft=0.5*rho*vtfps*vtfps;
% Convert to Pascals
% q=6894.76*qpsi;
if SI==1
q=47.8803*qpsft;
else
    q=qpsft;
end
