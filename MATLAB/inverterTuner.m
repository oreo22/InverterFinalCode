fsw=6000;
 fcutoff = fsw/pi;
 wc=2*pi*fcutoff;
 tau=1/wc;
%tau=0.5;
%invPlant=1/(1+s*tau);
Numerator = {[1]};
Denominator = {[tau 1]};
sys=tf(Numerator,Denominator);
% disp(sys);
X=stepinfo(sys)
%[C_pi,info] = pidtune(sys,'PI')
%Results: Kp = 1.04, Ki = 3.75e+04, if tau=0.5, it's 6.26 

%Based on params, this is the theoretical chararcteristics of sys
%  RiseTime: 1.8308e-04
%     SettlingTime: 3.2601e-04
%      SettlingMin: 0.9000
%      SettlingMax: 1.0000
%        Overshoot: 0
%       Undershoot: 0
%             Peak: 1.0000
%         PeakTime: 8.7882e-04


%So fcutoff = fsw/pi, wc=2*pi*fcutoff, and tau=1/wc
%filters Vout/Vin transfer function for fundamental frequencies is 1/(1+s*tau), that a first order low pass.  
%Remember, this holds when you update the duty cycles ONCE per period