

syms Kp Ti
eqn1 = sqrt(1*(2*x + y*T)/2 == 166.8776;
eqn2 = (-2*x + y*T)/2 == -166.3224;
sol = solve([eqn1, eqn2, eqn3], [x, y]);
KpSol = sol.x
KiSol = sol.y


T=1/6000;
syms x y
eqn1 = (2*x + y*T)/2 == 166.8776;
eqn2 = (-2*x + y*T)/2 == -166.3224;
sol = solve([eqn1, eqn2, eqn3], [x, y]);
KpSol = sol.x
KiSol = sol.y



%% Sine Table Generator
func = 'sin(2*pi*x)';
% Define the range over which to optimize breakpoints
xmin = 0;
xmax = 0.25;
% Define the data type and scaling for the inputs
xdt = ufix(16);
xscale = 2^-16;
% Define the data type and scaling for the outputs
ydt = sfix(16);
yscale = 2^-14;
% Specify the rounding method
rndmeth = 'Floor';
% Define the maximum acceptable error
%errmax = 2^-20;
% Choose even, power-of-2 spacing for breakpoints
spacing = 'pow2';
 % Specify the maximum number of points
nptsmax = 1040;
% Create the lookup table
[xdata,lookupTable,errworst] = fixpt_look1_func_approx(func, ...
xmin,xmax,xdt,xscale,ydt,yscale,rndmeth,[],nptsmax);
disp(length(lookupTable));

% Digital Filters
%% Notch Filter Generation
Fs=6000; %Sampling frequency = 50Khz
GridFreq=60; %Nominal Grid Frequency in Hz
Tfinal=2; %Time the simulation is run for = 0.5 seconds
Ts=1/Fs; %Sampling Time = 1/Fs
t=0:Ts:Tfinal; %Simulation Time vector
wn=2*pi*GridFreq; %Nominal Grid Frequency in radians
ws=2*pi*Fs;
[b,a]=butter(1,[2*pi*(115/(ws/2)) 2*pi*(125/(ws/2))],'stop'); %Enter the band you don't want the frequencies to pass through
%Ya do Fnotdesire/(ws/2)) because the method wants a ratio of the not-desired F over (ws/2)
[H,W]=freqz(b,a,1000);
disp(b);
disp(a);


figure(1);
subplot(2,1,1)
plot(W*Fs/2/pi,abs(H)); %restoring the frequencies to time domain, undoing the Fs/2 thing too
grid
xlabel('Frequency (Hz)')
ylabel('Magnitude')
subplot(2,1,2)
plot(W*Fs/2/pi,angle(H)*180/pi);
grid

f=120;
t=0:1/Fs:1/f-1/Fs;
x=sin(2*pi*f*t);
x_filtered=filter(b,a,x);

figure(2)
plot(t,x,t,x_filtered);


