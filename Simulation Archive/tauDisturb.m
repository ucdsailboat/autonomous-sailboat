% Calculations for the disturbance forces for the sailboat simulation.
% Utilizes parameters from the physical boat (beam, length, draft),
% parameters of the fluid (density), and models waves as a sinusoid whose 
% frequency is a function of the velocity of the waves as well as the
% boat velocity. 
%
% Inputs: wave frequency (wave_freq), boat speed (V_boat), wave height (h)
% Outputs: resultant forces and moments component-wise (X_d, Y_d, Z_d)
%
% Assumptions: typical "wave_freq" of lake is 2*pi*(vel/wavelength) = 10*pi,
% wave ht. 1/4m, 
%
% Next: update chi (wave incidence), link time dependent variables to
% Simulink 
%
% Bryan Zhao - 4/6/18

function [X_d, Y_d, Z_d] = tauDisturb(wave_freq, V_boat, x_0, t)

% define constants
T = 0.4064; % draft of boat [m]
L = 1.0541; % length [m]
rho = 1000; % density of water [kg/m^3]
B = 0.3333; % beam of boat [m]
g = 9.81; % std. gravity [m/s^2]
beta_c = 0; % rel. angle between heading and current [deg]
chi = 0; % wave incidence? need to redefine [deg]
h = 1/4; % wave height [m]

% dependencies
k = (4 * pi^2 * wave_freq^2)/ g; % wave number
enc_freq = wave_freq - k*V_boat*cos(chi); % encounter freq.

% wave profile
s = -k*h/2 * sin(k*x_0 - enc_freq*t); % wave slope
eta = (h/2) * cos(k*x_0 - enc_freq*t); % wave profile

X_d = rho * g * B * L * T * cos(beta_c) * s; % X component of tau_disturb
Y_d = -rho * g * B * L * T * sin(beta_c) * s;
Z_d = (1/24) * rho * g * B * L * (L^2 - B^2) * T * k^2 * sin(2*beta_c) * eta;


% end of function
