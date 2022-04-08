function [F_res] = LDRV_ResistanceForceDavis(A,B,C,v)
%LDRV_ResistanceForces Calculate the total resistance power.
% 
% Inputs:
% A = First Davis Equation Coefficient
% B = Second Davis Equation Coefficient
% C = Third Davis Equation Coefficient
% v = Vehicle Speed [m/s]
%
% Output:
% F_res = Resistance Force [N]

F_res = A+B*v+C*(v^2); %Total Resistance Force [N]

end

