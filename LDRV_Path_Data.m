% Script with data about the railway path & enviromental conditions.
% PLEASE, DO NOT RUN THIS SCRIPT, THIS IS REQUIRED IN "LDVR_Main.m". 
% RUN "LDVR_Main.m" ONLY.

%% Roma - Firenze Railway Path (255km)

% x_sl = (cat(1, 110*ones(5000,1), 165*ones(5000,1), 250*ones(235000,1),...
%     170*ones(5000,1), 130*ones(4000,1), 2*ones(1000,1)...
%     ))'; %Speed Limit Reference of driving path [km/h]

%DEMO - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
x_sl = (cat(1, 110*ones(1000,1), 165*ones(4000,1), 130*ones(10000,1),...
     2*ones(1000,1)))'; %Speed Limit Reference of driving path [km/h]
%END DEMO - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 
x = 1:length(x_sl); %x Coordinates References of driving path [m]

ini_alt = 10; %Over The Sea Altitude at the initial point [m] 

alpha = (cat(1, 1*ones(500,1), 2*ones(3000,1), 0*ones(1500,1),...
    -1*ones(10000,1), 0*ones(1000,1)))'; %Path Slope [degrees]

x_altitude = zeros(length(alpha),1); %Altitude Profile [m]
x_altitude(1) = ini_alt;

for i=2:length(alpha)
    x_altitude(i) = x_altitude(i-1)+tand(alpha(i));
end

wind = zeros(length(x_sl),1); %Wind Speed at the train direction [m/s]