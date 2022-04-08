function [a_traction,a_braking] = LDRV_AccelerationStudy(P_mr,T_mr,om_m,om_w,tau,...
    eta_d,eta_r,T_friction,a_max_l,Tb,om_smb,r_w,J_m,J_w,F_res,m_tot,n_w)
%AccelerationStudy Calculates maximum acceleration for both traction and
%braking case.
% 
% Inputs:
% P_mr = Rated Power [W]
% T_mr = Rated Torque [Nm]
% om_m = Angular Motor Speed [rad/s]
% om_w = Angular Wheel Speed [rad/s]
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
% F_res = Total Resistive Force [N]
% m_tot = Total Mass [kg]
% n_w = Number of Wheels
%
% Outputs:
% a_traction = Traction Acceleration [m/s^2]
% a_braking = Braking Acceleration [m/s^2]

%% Lambda Abstraction Functions

a_d = @(Tm) (((eta_d*Tm)/(r_w*tau))-F_res)/(m_tot+...
    ((eta_d*J_m)/(tau*r_w)^2)+((n_w*J_w)/(r_w^2))); %Direct Acceleration

a_b_w = @(Tb) ((-F_res)-(Tb/r_w))/(m_tot+((eta_d*J_m)/...
    ((r_w*tau)^2))+((n_w*J_w)/(r_w^2))); %Braking Acceleration on wheels

a_b_m = @(Tm) ((Tm/(r_w*tau))-(eta_r*F_res))/...
    ((eta_r*m_tot)+(J_m/((r_w*tau)^2))+...
    ((eta_r*n_w*J_w)/(r_w^2))); %Braking Acceleration on motor

%% Electric Motor Calculations

[~,T] = EMSModel(P_mr,T_mr,om_m);

%% Calculations

%Maximum Acceleration Available (Traction)
T_motor_trc = T;
T_wheels_trc = (T_motor_trc*eta_d)/tau; %Traction Torque on the wheel [Nm]
%check adhesion condition
if (T_wheels_trc/r_w) > T_friction %If traction force is bigger than...
    %the adhesion force...
    T_wheels_trc = T_friction*r_w;
end
T_motor_trc = (tau*T_wheels_trc)/eta_d; %Traction Torque [Nm]
a_traction = a_d(T_motor_trc); %Traction Acceleration [m/s^2]
%check comfort condition
if a_traction > a_max_l
    a_traction = a_max_l;
end

%Maximum Acceleration Available (Braking)
if om_w > om_smb
    %braking with the motor
    T_motor_brk = T; %Braking Motor Torque Available [Nm]
    T_wheels_brk = (T_motor_brk)/(eta_r*tau); %Braking Torque on the wheel [Nm]
    %check adhesion condition
    if (T_wheels_brk/r_w) > T_friction
        T_wheels_brk = T_friction*r_w;
    end
    T_motor_brk = tau*eta_r*T_wheels_brk; %Braking Torque [Nm]
    a_braking = a_b_m(T_motor_brk); %Braking Acceleration [m/s^2]
    %check comfort condition
    if a_braking > a_max_l
        a_braking = a_max_l;
    end
else
    %braking with the wheel brakes
    %check adhesion condition
    if (Tb/r_w) > T_friction
        Tb = T_friction*r_w;
    end
    a_braking = a_b_w(Tb); %Braking Acceleration [m/s^2]
    %check comfort condition
    if a_braking > a_max_l
        a_braking = a_max_l;
    end
end
a_braking = -a_braking;

end
 
