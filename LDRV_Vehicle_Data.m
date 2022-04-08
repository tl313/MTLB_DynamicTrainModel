% Script with data about the vehicle.
% PLEASE, DO NOT RUN THIS SCRIPT, THIS IS REQUIRED IN "LDVR_Main.m". 
% RUN "LDVR_Main.m" ONLY.

%% FS Class Elettro Treno Rapido ETR 500 (AnsaldoBreda)

%General
%This train has a max speed record of 362 km/h, 4DC motos (each of 1100kW)
%per locomotive, 2 locomotive and 10 carriages

%Motor and Transmission
T_mr = 8*6875; %Rated Torque [Nm]
om_mr = 160; %Rated Speed [rad/s]
P_mr = T_mr*om_mr; %Rated Power [W]
tau = 35/92; %Transmission Ratio
eta_d = 0.975; %Transmission Direct Efficiency
eta_r = 0.95; %Transmission Reverse Efficiency

%Masses
m_loco = 72000; %Locomotive Mass [kg]
m_car = 45400; %Carriage Mass [kg]
J_m = 8*10; %Motor Inertia Moment [kgm^2]
J_w = 90; %WheelSet Inertia Moment [kgm^2]

%Resistance Forces
f_r = 0.008; %Rolling Friction Coefficient
S = 7.8; %Vehicle Cross Section [m^2]
Cd = 1.05; %Drag Coefficient

%Davis Equation from Shinkansen Series 200
A = 8.202;
B = 0.10656;
C = 0.0119322;

%Brakes
Tb = 10000; %Braking Torque [Nm]
om_smb = 1; %Starting Wheel Speed of Mechanical Brakes Operation [rad/s]

%Misch
r_w = 0.445; %Wheel Radius [m]
n_loco = 2; %Number of Locomotives
n_car = 10; %Number of Carriages
n_w_loco = 8; %Number of Wheels per Locomotive
n_w_car = 8; %Number of Wheels per Carriage
mu = 0.99*0.35; %Rail Friction Limit

%% Others
g = 9.81; %Gravity [m/s^2]
rho = 1.25; %Air Density [kg/m^3]
a_max_l = 1.5; %Maximal Comfort Limit, Longitudinal Acceleration [m/s^2]
a_max_t = 1.5; %Maximal Comfort Limit, Lateral Acceleration [m/s^2]
