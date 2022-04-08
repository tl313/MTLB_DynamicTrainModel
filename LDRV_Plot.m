% Script with all the plots.
% PLEASE, DO NOT RUN THIS SCRIPT, THIS IS REQUIRED IN "LDVR_Main.m". 
% RUN "LDVR_Main.m" ONLY.

%% Settings

color_1 = '#085027';
color_2 = '#66717e';
color_3 = '#c17817';
color_bkg = '#fff2bd';
FS = 14; %Font Size
LW = 2; %Line Width

%% Initialize
close all

%% EMSM (Electric Motor Simulation Model)

if FLAG_01 == 1
    figure
    yyaxis left
    plot(rpm_vector,motor_torque,'color',color_1,'LineWidth',LW)
    grid on
    xlabel('Speed [rpm]','FontSize',FS)
    ylabel('Torque [Nm]','FontSize',FS)
    ylim([-10 T_mr*1.25])
    hold on
    plot(1000,T_1,'r*','MarkerSize',12)
    plot(2000,T_2,'r*','MarkerSize',12)
    yyaxis right
    plot(rpm_vector,motor_power,'color',color_2,'LineWidth',LW)
    grid on
    ylabel('Power [W]','FontSize',FS)
    title('#1 - Torque and Power')
    xlim([rpm_min-10 rpm_max+10])
    ylim([-10 P_mr*1.2])
    set(gca,'Color',color_bkg)
    hold on
    plot(1000,P_1,'g*','MarkerSize',12)
    plot(2000,P_2,'g*','MarkerSize',12)
end

%% Case Study

if FLAG_02 == 1
    figure
    plot(x,x_sl,'color',color_1,'LineWidth',LW)
    grid on
    xlabel('Space [m]','FontSize',FS)
    ylabel('Speed Limit [Kmh]','FontSize',FS)
    title('#2 - Path')
    xlim([-10 length(x)+10])
    ylim([-10 500])
    set(gca,'Color',color_bkg)
end

%% Speed, Acceleration and Power Profiles

if FLAG_03 == 1
    figure
    subplot(3,1,1)
    title('#3 - Speed, Acceleration and Power Profiles')
    plot(x,x_sl,'--','color',color_2,'LineWidth',LW-1)
    hold on
    plot(x,v_veh,'color',color_1,'LineWidth',LW)
    wind_kmh = wind*3.6; %Wind Speed at the train direction [km/h]
    plot(x,wind_kmh,'color',color_3,'LineWidth',LW-1)
    hold off
    legend('Speed Limit','Train Speed','Wind Speed')
    grid on
    xlabel('Space [m]','FontSize',FS)
    ylabel('Speed [Km/h]','FontSize',FS)
    set(gca,'Color',color_bkg)

    subplot(3,1,2)
    plot(x,a_veh,'color',color_1,'LineWidth',LW)
    grid on
    xlabel('Space [m]','FontSize',FS)
    ylabel('Acceleration [m/s^2]','FontSize',FS)
    hold on
    plot(x,a_max_l*ones(1,length(x)),'--',x,-a_max_l*ones(1,length(x))...
        ,'--','color',color_2,'LineWidth',LW-1)
    legend('Train Acceleration','Comfort Zone')
    ylim([-3 3])
    set(gca,'Color',color_bkg)

    subplot(3,1,3)
    plot(x,P_veh,'color',color_1,'LineWidth',LW)
    hold on
    plot(x,zeros(1,length(x)),'--','color',color_2,'LineWidth',LW-1)
    grid on
    xlabel('Space [m]','FontSize',FS)
    ylabel('Power [W]','FontSize',FS)
    set(gca,'Color',color_bkg)
end

%% Travel Profile

if FLAG_04 == 1
    figure
    title('#4 - Travel Profile')
    yyaxis left
    plot(x,x_sl,'--','color',color_2,'LineWidth',LW-1)
    hold on
    plot(x,v_veh,'-','color',color_1,'LineWidth',LW)
    wind_kmh = wind*3.6; %Wind Speed at the train direction [km/h]
    plot(x,wind_kmh,'color',color_3,'LineWidth',LW-1)
    hold off
    ylabel('Speed [Km/h]','FontSize',FS)
    yyaxis right
    plot(x,x_altitude,'color',color_2,'LineWidth',LW-1)
    ylabel('Altitude [m]','FontSize',FS)
    legend('Speed Limit','Train Speed','Wind Speed')
    grid on
    xlabel('Space [m]','FontSize',FS)
    set(gca,'Color',color_bkg)
end