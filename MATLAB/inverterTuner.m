fsw=6000;
 fcutoff = fsw/pi;
 wc=2*pi*fcutoff;
 tau=1/wc;
%invPlant=1/(1+s*tau);
Numerator = {[1]};
Denominator = {[tau 1]};
sys=tf(Numerator,Denominator);
disp(sys);
[C_pi,info] = pidtune(sys,'PI')
%Results: Kp = 1.04, Ki = 3.75e+04 

%So fcutoff = fsw/pi, wc=2*pi*fcutoff, and tau=1/wc
%filters Vout/Vin transfer function for fundamental frequencies is 1/(1+s*tau), that a first order low pass.  
%Remember, this holds when you update the duty cycles ONCE per period