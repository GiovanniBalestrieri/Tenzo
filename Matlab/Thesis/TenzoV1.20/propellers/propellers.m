clear all;
chord=0.10;
pitch=0.0;
dia=1.6;
R=dia/2.0;
RPM=2100.;
%pitch angle setting at tip
tip=25.0;
xt=R;
%pitch angle setting at 25% radius
hub=65.0;
xs=0.1*R
tonc=0.12*chord;
rho=1.225;
n=RPM/60.0;
omega=n*2.0*pi;
coef1=(tip-hub)/(xt-xs);
coef2=hub-coef1*xs;
rstep=(xt-xs)/10
r1=[xs:rstep:xt];
k=0;
eff0=0;
V=60;
thrust=0.0;
torque=0.0;
for j=1:size(r1,2),
 rad=r1(j);
 theta=coef1*rad+coef2+pitch;
 t2(j)=theta;
 th=theta/180.0*pi;
 sigma=2.0*chord/2.0/pi/rad;
 a=0.1;
 b=0.01;
 finished=0;
 sum=1;
 while (finished==0),
  V0=V*(1+a);
  V2=omega*rad*(1-b);
  phi=atan2(V0,V2);
  alpha=th-phi;
  cl=6.2*alpha;
  cd=0.008-0.003*cl+0.01*cl*cl;
  Vlocal=sqrt(V0*V0+V2*V2);
  DtDr=0.5*rho*Vlocal*Vlocal*2.0*chord*(cl*cos(phi)-cd*sin(phi));
  DqDr=0.5*rho*Vlocal*Vlocal*2.0*chord*rad*(cd*cos(phi)+cl*sin(phi));
  tem1=DtDr/(4.0*pi*rad*rho*V*V*(1+a));
  tem2=DqDr/(4.0*pi*rad*rad*rad*rho*V*(1+a)*omega);
  anew=0.5*(a+tem1);
  bnew=0.5*(b+tem2);
  if (abs(anew-a)<1.0e-5),
   if (abs(bnew-b)<1.0e-5),
    finished=1;
   end;
  end;
  a=anew;
  b=bnew;
  sum=sum+1;
  if (sum>500),
   finished=1;
  end;
 end;
 a2(j)=a;
 b2(j)=b;
 thrust=thrust+DtDr*rstep;
 torque=torque+DqDr*rstep;
end;
disp(thrust);
disp(torque);
t=thrust/(rho*n*n*dia*dia*dia*dia);
q=torque/(rho*n*n*dia*dia*dia*dia*dia);
J=V/(n*dia);
if (t<0),
 eff=0.
else
 eff=t/q*J/(2.0*pi)
end;