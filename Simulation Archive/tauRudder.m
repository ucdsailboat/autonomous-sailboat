% Control Surface Force: Rudder
function [X_r, Y_r, N_r] = tauRudder(betaP_ac, V_c, delta_r)
% Equations for tauRudder, control surface force of the rudder 
% betaP_ac [deg]: direction of the apparent water velocity, measured clockwise
% from the positive x-axis of the body (opposite of boat velocity) 
% V_c [m/s]: magnitude of the apparent water velocity (same as boat
% velocity)
% delta_r [deg]: angle of the rudder 

%load('Sailboat_Constants_1')   % for testing

% based on Equation 5.5: Equations for Lift & Drag 
L_r = C_lr*0.5*rho_c*(V_c^2)*S_r;
D_r = C_d*0.5*rho_c*(V_c^2)*S_r;

% based on Table 5.1: Direction of the Lift force in the rudder
if (betaP_ac > delta_r) && ((delta_r + 90) > betaP_ac)
    betaPl_ac = betaP_ac + 90/2;
elseif (betaP_ac > delta_r + 90) && ((delta_r + 2*90) > betaP_ac)
    betaPl_ac = betaP_ac - 90/2;
end 

% based on Equation 5.9
X_r = - abs(D_r)*cosd(betaP_ac) + abs(L_r)*cosd(betaPl_ac);
Y_r = abs(D_r)*sind(betaP_ac) + abs(L_r)*sind(betaPl_ac);
N_r = (b_r/4)*Y_r;
end 
