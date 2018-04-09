% Body Forces: Rigid Body, Added Mass, and Damping 
function [X_b, Y_b, N_b] = tauBody(betaP_rc, V_rc, r_rc, udot_rc, vdot_rc, rdot)
% Equations for the hydrodynamic body forces on the hull due to the rigid
% body, the added mass, and damping 
% betaP_rc [deg]: direction of the relative sailboat velocity to the current
% (current assumed to have a negligible velocity) 
% V_rc [m/s]: magnitude of the relative sailboat velocity
% r_rc [rad/s]: yaw rate of the sailboat in the z-direction
% udot_rc [m/s^2]: magnitude of the sailboat acceleration in the x-direction
% vdot_rc [m/s^2]: magnitude of the sailboat acceleration in the y-direction
% rdot [rad/s^2]: yaw acceleration of the sailboat in the z-direction

%load('Sailboat_Constants_1')       %for testing

% based on Table 5.4 and Equation 5.15: Dimensionless velocities 
uP_rc = cosd(betaP_rc);
vP_rc = sind(betaP_rc);
rP_rc = r_rc*L/V_rc;

% based on Equation 5.14: Equations for the rigid body forces (A_ut & A_ul
% TBD)
X_rb = 0.5*rho_c*A_ut*V_rc^2*(XP_uu*uP_rc^2 + XP_vv*vP_rc^2 + XP_ur*uP_rc*rP_rc + XP_vr*vP_rc*rP_rc + XP_rr*rP_rc^2);
Y_rb = 0.5*rho_c*A_ul*V_rc^2*(YP_v*vP_rc + YP_vv*vP_rc*abs(vP_rc) + YP_r*rP_rc + YP_vr*vP_rc*rP_rc + YP_rr*rP_rc*abs(rP_rc));
N_rb = 0.5*rho_c*A_ul*L^3*V_rc^2*(NP_v*vP_rc + NP_r*rP_rc + NP_vrr*vP_rc*rP_rc^2 + NP_vvr*vP_rc^2*rP_rc + NP_rr*rP_rc*abs(rP_rc));

% based on Equation 5.15: Components of sailboat velocity
u = V_rc*cosd(betaP_rc);       % u_rc = u
v = V_rc*cosd(betaP_rc);       % v_rc = v     
r = r_rc; 

% based on Equation 5.26: Equations for the added mass forces
X_a = X_udot*udot_rc - Y_vdot*v*r; 
Y_a = Y_vdot*vdot_rc + X_udot*u*r;
N_a = N_rdot*rdot + Y_vdot*v*u - X_udot*u*v;

% based on Equation 5.30 and Equation 5.33: Equations for the damping forces
X_da = -X_u*u;
Y_da = -Y_v*v - Y_r*r;
N_da = -N_v*v - N_r *r;

X_b = X_rb + X_a + X_da;
Y_b = Y_rb + Y_a + Y_da;
N_b = N_rb + N_a + N_da;
end 