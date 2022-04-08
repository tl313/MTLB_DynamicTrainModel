function [P,T] = EMSModel(P_mr,T_mr,omega)
    %EMSModel Calculate power and torque of a motor, given the rotational speed.
    % 
    % Inputs:
    % P_mr = Rated Motor Power [W]
    % T_mr = Rated Motor Torque [Nm]
    % omega = Rotational Speed [rad/s]
    %
    % Outputs:
    % P = Motor Power [W]
    % T = Motor Torque [Nm]
    
    omega_rated = P_mr/T_mr;
    if omega<omega_rated
        T = T_mr;
        P = T_mr*omega;
    else
        P = P_mr;
        T = P_mr/omega;
    end
end