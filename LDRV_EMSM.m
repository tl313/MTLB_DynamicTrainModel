% Script with the Electric Motor Simulation Model.
% PLEASE, DO NOT RUN THIS SCRIPT, THIS IS REQUIRED IN "LDVR_Main.m". 
% RUN "LDVR_Main.m" ONLY.

%The electric motor can be characterized by two modes of operation: a
%constant-torque mode below the rated speed, and a constant-power mode
%above.

%% Calculations for the plot

rpm_vector = linspace(rpm_min,rpm_max,n_it);
rads_vector = rpm_vector/9.549297;
omega_rated_rad = P_mr/T_mr; %Rated (or Nominal) Speed [rad/s]
omega_rated = 9.549297*omega_rated_rad; %Rated (or Nominal) Speed [rpm]

motor_torque = zeros(length(rpm_vector),1); %Motor Torque values [Nm]
motor_power = zeros(length(rpm_vector),1); %Motor Power values [W]

for i=1:length(rpm_vector)
    if rpm_vector(i)<omega_rated
        motor_torque(i) = T_mr;
        motor_power(i) = T_mr*rads_vector(i);
    else
        motor_power(i) = P_mr;
        motor_torque(i) = P_mr/rads_vector(i);
    end
end