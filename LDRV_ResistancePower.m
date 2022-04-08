function [P_res,F_res] = LDRV_ResistancePower(m_loco,m_car,n_loco,n_car,alpha,...
    f_r,g,rho,Cd,S,v,wind)
%LDRV_ResistanceForces Calculate the total resistance power and force.
% 
% Inputs:
% m_loco = Locomotive Mass [kg]
% m_car = Carriage Mass [kg]
% n_loco = Number of Locomotives
% n_car = Number of Carriages
% alpha = Slope [degrees]
% f_r = Rolling Friction Coefficient
% g = Gravity Acceleration
% rho = Air Density [kg/m^3]
% Cd = Drag Coeficcient
% S = Vehicle Cross Section [m^2]
% v = Vehicle Speed [m/s]
% wind = Wind Speed at the train direction [m/s]
%
% Output:
% P_res = Resistance Power (negative) [W]
% F_res = Resistance Force (negative) [N]

m_tot = m_loco*n_loco + m_car*n_car; %Total Mass [kg]
P_weight = -m_tot*g*sind(alpha)*v; %Weight Resistance Power [W]
P_roll = -f_r*m_tot*g*cosd(alpha)*v; %Roll Resistance Power [W]
P_aero = -0.5*rho*S*Cd*((v-wind)^3); %Aero Resistance Power [W]

P_res = P_weight+P_roll+P_aero; %Total Resistance Power [W]
if P_res == 0
    F_res = -m_tot*g*sind(alpha)-f_r*m_tot*g*cosd(alpha);
else
    F_res = P_res/v;
end
end

