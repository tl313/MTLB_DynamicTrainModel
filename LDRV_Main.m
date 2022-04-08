% Main Script - Longitudinal Dynamics of a Railway Vehicle
%
% THIS IS THE MAIN SCRIPT. RUN THIS ONLY.
%
% This is the main script, it is important to run only this: any other
% scripts required will be automatically started and invoked.

%% Initialize all data

clear
close all
clc

%% FLAGS

% %0 is FALSE, 1 is TRUE
FLAG_01 = 0; %Setting Flag for Motor Characteristic Curve Plot
FLAG_02 = 0; %Setting Flag for Speed Limit Profile Plot
FLAG_03 = 0; %Setting Flag for Speed-Acceleration-Power Profiles Plot
FLAG_04 = 1; %Setting Flag for Travel Profile Plot

%% Vehicle Data

run('LDRV_Vehicle_Data.m')

%% Path Data

run('LDRV_Path_Data.m')

%% Electric Motor Simulation Model

rpm_min = 0; %Minimum Rotational Speed [rpm]
rpm_max = 10000; %Maximum Rotational Speed [rpm]
n_it = 1000; %Study Points number (for plot #1 only)

run('LDRV_EMSM.m')

%Punctual Verification of the Model
[P_1,T_1] = EMSModel(P_mr,T_mr,1000*0.10472);
[P_2,T_2] = EMSModel(P_mr,T_mr,2000*0.10472);

%% Driving Control Law

run('LDRV_DCL.m')

%% Plots

run('LDRV_Plot.m')
