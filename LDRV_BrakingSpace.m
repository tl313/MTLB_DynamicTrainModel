function [braking_space] = LDRV_BrakingSpace(v_1,v_2,P_mr,T_mr,tau,...
    eta_d,eta_r,T_friction,a_max_l,Tb,om_smb,r_w,J_m,J_w,m_tot,...
    n_w,alpha,f_r,g,rho,Cd,S,x_init,wind)
%LDRV_BrakingSpace Calculates the required braking space.
% 
% Inputs:
% v_1 = Initial Linear Speed [m/s]
% v_2 = Desired Final Linear Speed [m/s]
% P_mr = Rated Power [W]
% T_mr = Rated Torque [Nm]
% tau = Transmission Ratio
% eta_d = Transmission Direct Efficiency
% eta_r = Transmission Reverse Efficiency
% T_friction = Maximum wheel force to respect adhesion [N]
% a_max_l = Maximal Comfort Limit, Longitudinal Acceleration [m/s^2]
% Tb = Braking Torque [Nm]
% om_smb = Starting Operation Speed of Mechanical Brakes [rad/s]
% r_w = Wheel Radius [m]
% J_m = Motor Inertia Moment [kgm^2]
% J_w = WheelSet Inertia Moment [kgm^2]
% m_tot = Total Mass [kg]
% n_w = Number of Wheels
% alpha = VECTOR Average Slope [degrees]
% f_r = Rolling Friction Coefficient
% g = Gravity Acceleration
% rho = Air Density [kg/m^3]
% Cd = Drag Coeficcient
% S = Vehicle Cross Section [m^2]
% x_init = Initial Distance [m]
% wind = VECTOR Wind Speed at the train direction [m/s]
%
% Output:
% braking_space = Required Braking Space [m]

%% Calculations

braking_space = 0;

while v_1 > v_2
    %Initial Speed Calculations
    om_w = v_1/r_w; %Angular Wheel Speed [rad/s]
    om_m = om_w/tau; %Angular Motor Speed [rad/s]

    %Resistance Calculations
    P_weight = -m_tot*g*sind(alpha(x_init+braking_space))*v_1; %Weight Resistance Power [W]
    P_roll = -f_r*m_tot*g*cosd(alpha(x_init+braking_space))*v_1; %Roll Resistance Power [W]
    P_aero = -0.5*rho*S*Cd*((v_1-wind(x_init+braking_space))^3); %Aero Resistance Power [W]
    P = P_weight+P_roll+P_aero; %Total Resistance Power [W]
    if P == 0
        F_res = -m_tot*g*sind(alpha(x_init+braking_space))-...
            f_r*m_tot*g*cosd(alpha(x_init+braking_space));
    else
        F_res = P/v_1; %Total Resistance Force [N]
    end
    
    %Maximum Acceleration Available (Braking)
    [~,a_braking] = LDRV_AccelerationStudy(P_mr,T_mr,om_m,om_w,tau,...
    eta_d,eta_r,T_friction,a_max_l,Tb,om_smb,r_w,J_m,J_w,F_res,m_tot,n_w);

    %Speed Deceleration
    v_1 = sqrt((v_1)^2+(2*a_braking));
    braking_space = braking_space+1;
end
end

