% Control Surface Force: Sail 
function [X_s,  Y_s,  N_s] = tauSail(betaP_aw, V_w, delta_s)
% Equations for tauSail, control surface force of the sail
% betaP_aw [deg]: direction of the apparent wind velocity, measured clockwise
% from the positive x-axis of the body
% V_w [m/s]: magnitude of the apparent wind velocity 
% delta_s [deg]: angle of the sail

%load('Sailboat_Constants_1')       %for testing

% based on Equation 5.5: Equations for Lift & Drag 
L_s = C_ls*0.5*rho_w*(V_w^2)*S_s;
D_s = C_d*0.5*rho_w*(V_w^2)*S_s;

% based on Table 5.2: Direction of the Lift force in the Sail 
if (betaP_aw > delta_s ) && ((delta_s + 90) > betaP_aw)
    betaPl_aw = betaP_aw + 90/2;
elseif (betaP_aw > delta_s + 90) && ((delta_s + 2*90) > betaP_aw)
    betaPl_aw = betaP_aw - 90/2;
end

% based on Equation 5.11
X_s = abs(D_s)*cosd(betaP_aw) + abs(L_s)*cosd(betaPl_aw);
Y_s = abs(D_s)*sind(betaP_aw) + abs(L_s)*sind(betaPl_aw);
N_s = (b_s/4)*Y_s;
end 
