% Script with all the calculationts about the driving simulation. Driving Control Law.
% PLEASE, DO NOT RUN THIS SCRIPT, THIS IS REQUIRED IN "LDVR_Main.m". 
% RUN "LDVR_Main.m" ONLY.

%% Settings

delta_V = 2; %Hysteresis Speed Rate [m/s]
a_brk_star = 0.5; %Acceleration to study the Braking Distance only if required [m/s^2]

%% Initialize Useful Data

v_veh = zeros(length(x),1); %Linear Vehicle Speed [m/s]
a_veh = zeros(length(x),1); %Linear Vehicle Acceleration [m/s^2]
P_veh = zeros(length(x),1); %Vehicle Power [W]
T_veh = zeros(length(x),1); %Vehicle Torque [Nm]
F_z = zeros(length(x),1); %Vertical Force on the wheel [N]
om_w = zeros(length(x),1); %Angular Wheel Speed [rad/s]
om_m = zeros(length(x),1); %Angular Motor Speed [rad/s]
T_friction = zeros(length(x),1); %Maximum Wheel Torque to respect adhesion [Nm]
P_res = zeros(length(x),1); %Total Resistive Power [W]
F_res = zeros(length(x),1); %Total Resistive Force [N]
a_traction = zeros(length(x),1); %Maximum Traction Acceleration [m/s^2]
a_braking = zeros(length(x),1); %Maximum Braking Acceleration [m/s^2]
next_sl = zeros(length(x),1); %Speed Limit of the next sector [Kmh]
next_sl_loc = zeros(length(x),1); %Location of next Speed Limit Change [m]
bd = zeros(length(x),1); %Required Braking Distance [m]
bda = zeros(length(x),1); %Available Braking Distance [m]
Data_Matrix = zeros(length(x),14); %Results (for debugging purposes only)
deltaETA = 0;

%% Calculations

m_tot = m_loco*n_loco + m_car*n_car; %Total Mass [kg]
n_w = 4; %(n_w_loco*n_loco + n_w_car*n_car)/2; %Number of WheelSets
cp = find(diff(x_sl)); %Points Location where the speed limit changes [m]
x_sl = x_sl/3.6; %Speed Limit Reference of driving path [m/s]

for i=1:length(x)
    F_z(i) = ((m_tot*g*cosd(alpha(i)))/n_w); %Vertical Force [N]
    T_friction(i) = F_z(i)*mu; %Maximum wheel force to respect adhesion [N]
    Data_Matrix(i,13) = T_friction(i);
end

for i = 2:length(x)
   if max(cp)>x(i-1) %If whe are NOT at the last sector
        next_sl_loc(i) = cp(find(cp>x(i-1),1)); %Next Speed Limit Location [m]
        next_sl(i) = x_sl(next_sl_loc(i)+1); %Next Speed Limit [m/s]
    else
        next_sl_loc(i) = length(x);
        next_sl(i) = 0; %We assume that the vehicle must STOP at the end
    end
end

total_iteration = length(x)-1;
iteration = 0;
percentage = 0;
simplewaitbar = uifigure;
d_simplewaitbar = uiprogressdlg(simplewaitbar,...
    'Title','DCL Calculation Progress...',...
        'Message','1','Cancelable','on');
drawnow
tic

for i=2:length(x)

    if d_simplewaitbar.CancelRequested
        break
    end
    
    om_w(i-1) = v_veh(i-1)/r_w; %Angular Wheel Speed [rad/s]
    om_m(i-1) = om_w(i-1)/tau; %Angular Motor Speed [rad/s]
    
    %Resistance Study - - - - - - - - - - - - - - - - - - 
    [P_res(i-1),F_res(i-1)] = LDRV_ResistancePower(m_loco,m_car,n_loco...
        ,n_car,alpha(i-1),f_r,g,rho,Cd,S,v_veh(i-1),wind(i-1));
    Data_Matrix(i-1,14) = F_res(i-1);
    %Resistance Study - END - - - - - - - - - - - - - - -
    
    %Acceleration Study - - - - - - - - - - - - - - - - - 
    [a_traction,a_braking] = LDRV_AccelerationStudy(P_mr,T_mr,om_m(i-1),...
        om_w(i-1),tau,eta_d,eta_r,T_friction(i-1),a_max_l,Tb,...
        om_smb,r_w,J_m,J_w,F_res(i-1),m_tot,n_w);
    %Acceleration Study - END - - - - - - - - - - - - - -
    
    %Braking Space Study - - - - - - - - - - - - - - - - -
    bda(i-1) = next_sl_loc(i)-x(i-1); %Braking Distance Available [m]
    if next_sl(i-1)>x_sl(i-1) %If next Speed Limit > actual Speed Limit then 
        %the braking distance is not required
        bd(i-1) = -100; %DUMMY VALUE
    else %Calculate the BD only "near" the speed limit change
        bd_star = 0.5*(v_veh(i-1)^2-next_sl(i-1)^2)/a_brk_star;
        if bda(i-1)<bd_star
            bd(i-1) = LDRV_BrakingSpace(v_veh(i-1),next_sl(i-1),P_mr,T_mr,tau,...
            eta_d,eta_r,T_friction,a_max_l,Tb,om_smb,r_w,J_m,J_w,...
            m_tot,n_w,alpha,f_r,g,rho,Cd,S,x(i-1),wind);
        end
    end
    %Braking Space Study - END - - - - - - - - - - - - - -
    
    if bd(i-1) < bda(i-1)
        if v_veh(i-1)<x_sl(i-1)
            %Traction!
            %We will assume an uniformly accelerated motion between one 
            %point and the successive one (so each meter of distance).
            a_veh(i) = a_traction;
            v_veh(i) = sqrt((v_veh(i-1))^2+(2*a_veh(i)));
            om_motor = (v_veh(i)/r_w)/tau; %Angular Motor Speed [rad/s]
            [P_veh(i),T_veh(i)] = EMSModel(P_mr,T_mr,om_motor);
        else
            %Coasting!
            if v_veh(i-1)-x_sl(i-1)<delta_V
                a_veh(i) = 0;
                v_veh(i) = sqrt((v_veh(i-1))^2 +(2*a_veh(i)));
                [P_RPModel,~] = LDRV_ResistancePower(m_loco,m_car,n_loco,...
                    n_car,alpha(i),f_r,g,rho,Cd,S,v_veh(i),wind(i));
                P_veh(i) = -P_RPModel;
                om_motor = (v_veh(i)/r_w)/tau; %Angular Motor Speed [rad/s]
                T_veh(i) = P_veh(i)/om_motor;
            end
        end
    else
        %Braking!
        a_veh(i) = a_braking;
        if (v_veh(i-1))^2 +(2*a_veh(i))<0
            v_veh(i) = 0;
        else
            v_veh(i) = sqrt((v_veh(i-1))^2 +(2*a_veh(i)));
        end
        om_motor = (v_veh(i)/r_w)/tau; %Angular Motor Speed [rad/s]
        [P_EMSModel,T_veh(i)] = EMSModel(P_mr,T_mr,om_motor);
        P_veh(i) = -P_EMSModel;
    end
    
    %Entering data inside of the data matrix - - - - - - -
    Data_Matrix(i-1,1) = x(i-1);
    Data_Matrix(i-1,2) = x_sl(i-1);
    Data_Matrix(i-1,3) = v_veh(i-1);
    Data_Matrix(i-1,4) = a_veh(i-1);
    Data_Matrix(i-1,5) = next_sl(i-1);
    Data_Matrix(i-1,6) = next_sl_loc(i-1);
    Data_Matrix(i-1,7) = bda(i-1);
    Data_Matrix(i-1,8) = bd(i-1);
    Data_Matrix(i-1,9) = a_traction;
    Data_Matrix(i-1,10) = a_braking;
    Data_Matrix(i-1,11) = P_veh(i-1);
    Data_Matrix(i-1,12) = T_veh(i-1);
    %Entering data inside of the data matrix - END - - - - -
    
    %Script for the visual message - - - - - - - - - - - - -
    iteration = iteration+1; 
    percentage = (iteration/total_iteration);
    if percentage > 0.15 && percentage < 0.35
        d_simplewaitbar.Title = 'Train is departing...';
    end
    if percentage > 0.35 && percentage < 0.75
        d_simplewaitbar.Title = 'Train is on time!';
    end
    if percentage > 0.75
        d_simplewaitbar.Title = 'Train is arriving at destination';
    end
    d_simplewaitbar.Value = percentage;
    if rem(iteration,20) == 0 %So every 20 iterations...
        percentage_r = 1-percentage;
        deltaETA = (toc*percentage_r)/percentage; %Remaining Time [s]
    end
    message = sprintf...
        ('Iterations: %d%s \nPercentage: %d%s \nTime: %d%s \nRemaining Time: %d%s',...
        iteration,' it',round(percentage*100),' %',round(toc),...
        ' s',round(deltaETA),' s');
    d_simplewaitbar.Message = message;
    %Script for the visual message - END - - - - - - - - - -
end

%Entering data inside of the data matrix - - - - - - - -
colNames = {'x','Speed Limit [m/s]','Speed [m/s]',...
        'Acceleration [m/s^2]','Next SL [m/s]'...
        'Next SL Loc [m]','BDA [m]','BD [m]','a traction [m/s^2]'...
        ,'a braking [m/s^2]','Power [W]','Torque [Nm]',...
        'Friction Torque [Nm]','Resistive Force [N]'};
Data_Table = array2table(Data_Matrix,'VariableNames',colNames);
%Entering data inside of the data matrix - END - - - - -

v_veh = v_veh*3.6; %Vehicle Speed [km/h]
x_sl = x_sl*3.6; %Speed Limit Reference of driving path [km/h]
close(d_simplewaitbar)
