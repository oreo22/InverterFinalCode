C=400*10^-6;%1500*10^-6;
L=4*10^-3;%(1.5*10^-3) +(100*10^-6);
w0=1/sqrt(L*C);
w=2*pi*60;
t=sqrt(6/(w0^2+w^2));
Vm=120;%*1.414; %*sin(w*t)
Vmax=(Vm*sin(w*t))/(1-(w/w0));
%Vmax=(Vm/(1-(w/w0)^2))*(sin(w*t)-(w/w0)*sin(w0*t));
Imax=((Vm*w*C)/(1-((w/w0)^2)))*(cos(w*t)-cos(w0*t));

disp(Imax);
disp(Vmax);

C=1500*10^-6;
L=(1.5*10^-3) +(100*10^-6);
w0=1/sqrt(L*C);
w=2*pi*60;
t=sqrt(6/(w0^2+w^2));
Vm=30; %*sin(w*t)
Vmax=(Vm*sin(w*t))/(1-(w/w0));
%Vmax=(Vm/(1-(w/w0)^2))*(sin(w*t)-(w/w0)*sin(w0*t));
Imax=((Vm*w*C)/(1-((w/w0)^2)))*(cos(w*t)-cos(w0*t));

disp(Imax);
disp(Vmax);

R=(Vm*sqrt(2))/3