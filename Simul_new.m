% close all;
clear all;
tic 
%% Constants Definition 
global gear_ratio gear_ratioT m mu mur rs rl J_motor J_loads GradAngle ddqpass Tor_dr
global curLoad maxS loadatM dq_global ddq_global Desired_M v_prev testLoad
global GradAngle2 GradAngle3 GradAngle4 GradAngle5 rs2 rs3 rs4 rs5

load transformtorq6
time_transform_ex = transformtorq6(211:410,2); 
torque_transform_ex = transformtorq6(211:410,1) ; %expanding 
torque_tr_ex = downsample(torque_transform_ex,2);
t_tr = 0.0054:0.0054:0.54;
time_transform_co = transformtorq6(2:101,2); %contracting
torque_tr_co = transformtorq6(2:101,1); %contracting  


gear_ratio = 1/33 * 19/43;
gear_ratioT = 1/198;
bl = 0.32;
bh = 0.19;

no_load_current = 213*10^-3;  %A
tor_constant = 19.9; %mNm/A 
speed_constant = 479*2*pi/60; %rad/Vs
no_load_speed = 8590; %rpm 
J_motor = 35.9*10^-7; %kgm^2
MR = no_load_current * tor_constant;

speed_constant_tr = 250;
tor_constant_tr = 39.227;


rad2rpm = 30/pi;
rpm2rad = pi/30;
rad2deg = 180/pi;
deg2rad = pi/180;
rmin = 0.06;
rmax = 0.113;

grd = -rmin; %ground position; touching with the wheel in the beginning
mu = 0.8;
mur = 0.01;
g = 9.81;
m1 = 1.2;
m2 = 1.2 + 1 + 0.26; 
m = 7.5 * 0.5; %4 
rs = rmin;
rl = rmax;
rs2 = rmin;
rs3 = rmin;
rs4 = rmin;
rs5 = rmin;

J_loads = 0.5 * m * rs^2;
total_Loads = J_loads + J_motor/(gear_ratio^2) + 859.714*10^-9*(43/30)^2;

%% Velocity Profile 

maxS = no_load_speed;
Trapezoidal = 2 ;
WLTP = 1 ;
Random = 2;

if Trapezoidal == 1 %Trapezoidal
    reachvel = 0.7;
    dt = 0.01;
    total_time = 10; 
    reachtime = total_time * 0.25;
    t = 0:dt:total_time;
    [toFind Index1] = min(abs(t-reachtime)); 
    [toFind Index2] = min(abs(t-(total_time-reachtime))); % last 25%
    accel = reachvel/reachtime * t(1:Index1);
    decel = reachvel -reachvel/reachtime*t(1:length(t)-Index2);  
    velocity_d = accel;
    velocity_d(Index1:Index2) = reachvel;
    velocity_d(Index2+1:length(t)) = decel;
    accel_d = diff(velocity_d)/dt; 
    accel_d(end+1) = 0; 
elseif WLTP == 1
    dt = 0.01;
    return_value = getWLTP();
    velocity_d = return_value(2,:);
    accel_d = diff(velocity_d)/dt;
    accel_d(end+1) = 0;
    t = return_value(1,:);
elseif Random == 1
    reachvel = 0.7;
    dt = 0.01;
    total_time = 14; 
    reachtime = total_time * 0.20;
    t = 0:dt:total_time;
    [toFind Index1] = min(abs(t-reachtime)); 
    [toFind Index2] = min(abs(t-reachtime*3)); % last 25 minus 4sec%
    Index3 = 1121;
    
    accel = reachvel/reachtime * t(1:Index1);
    decel = reachvel*4 -reachvel/reachtime * t(Index2+1:Index3);  
    velocity_d = accel;
    velocity_d(Index1:Index2) = reachvel;
    velocity_d(Index2+1:Index3) = decel;
    velocity_d(Index3+1:length(t)) = 0;
    accel_d = diff(velocity_d)/dt; 
    accel_d(end+1) = 0;
    figure(5)
    plot(t,velocity_d);
    
end

%% Path Generation (for animation I guess) 

%% Simulation Parameter Definition/Initialization 
Eff_rMin(1) = 1;
Eff_rMax(1) = 1;

cur_dr = 0;
Tor_dr = 0;
py = 0;
GradAngle = 12;
v_prev = 0;

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

Wheel_q(1) = 0; Wheel_dq(1) = 0; Wheel_ddq(1) = 0; transvel(1) = 0;
Mot_dq(1) = 0; Mot_ddq(1) = 0;
xDis(1) = bl/2; yDis(1) = rs + bh/2;
X(1,:) = [Wheel_q(1) Wheel_dq(1) xDis(1) yDis(1)];
% X2(1,:) = [q_carrier(1) dq_carrier(1)];
Mot_pid_dr(1,:) = zeros(1,3);

current_dr(1) = 0;
torques(1) = 0; 

radius_efficiency_actual(1,:) = [rmin 0];
loads(1) = 0;

Desired_Efficiency(1) = 0;
Actual_Efficiency(1) = 0;
rd(1) = rmin;
r(1) = rmin;
omega_d(1) = 0;
Mot_des_dq(1) = 0;
Motor_load(1) = 0;
del_r_max = (rmax-rmin)/53;
Load_from_Eff(1) = 0;
Load_from_Eff_currentR(1) = 0;
dq_global = 0;
ddq_global = 0;
Max_Force = mu * 9.81 * m * cosd(GradAngle); 
EtransformCumul = 0;

GradAngle2 = 6;
GradAngle3 = 9;
GradAngle4 = 12;
GradAngle5 = 15;
rd2(1) = rmin;
rd3(1) = rmin;
rd4(1) = rmin;
rd5(1) = rmin;
d_ef2(1) = 0;
d_ef3(1) = 0;
d_ef4(1) = 0;
d_ef5(1) = 0;

forIndex = rmin:0.001:rmax;

%% Run
for i=1:length(t)-1
    tspan = [t(i) t(i+1)]; %updates at every step
    radius_efficiency = eff_theory(velocity_d(i+1)/gear_ratio, GradAngle);
    
    Desired_Efficiency(i+1) = radius_efficiency(2); %maximum efficiency
    rd(i+1) = radius_efficiency(1); %desired Radius 
    
%     rd2(i+1) = radius_efficiency(3);
%     d_ef2(i+1) = radius_efficiency(4);
%     
%     rd3(i+1) = radius_efficiency(5);
%     d_ef3(i+1) = radius_efficiency(6);
%     
%     rd4(i+1) = radius_efficiency(7);
%     d_ef4(i+1) = radius_efficiency(8);
%     
%     rd5(i+1) = radius_efficiency(9);
%     d_ef5(i+1) = radius_efficiency(10);
    
%     radius_efficiency_actual(i+1,:) = [r(i) 

    if abs(rd(i+1) - r(i)) > del_r_max
        r(i+1) = r(i) + sign(rd(i+1) - r(i) ) * del_r_max;
    else
        r(i+1) = rd(i+1);
    end
%     r(i+1) = rmin;
%     r(i+1) = rmax;
%     r(i+1) = rd(i+1);
%     r(i+1) = r(i);

    %% transformation energy 
%     [~, kk] = min(abs(forIndex - r(i+1))); 
    if (round(rd(i+1)-r(i),6)) > 0 %ext 
        if round((r(i)-rmin)/0.000563+1) ~= 101
            EtransformCumul = EtransformCumul + ...
                (torque_tr_ex(round((r(i)-rmin)/0.000563+1))/1000 * ...
                (r(i+1)-r(i))/0.0563*45*pi/180);
        else
            EtransformCumul = EtransformCumul + ...
                (torque_tr_ex(100)/1000 * (r(i+1)-r(i))/rmin*45*pi/180); 
%             disp('its 101');
        end
    elseif round(rd(i+1)-r(i),6) < 0 %cont
        if round((r(i)-rmin)/0.000563) ~= 0 
            EtransformCumul = EtransformCumul + ...
                (torque_tr_co(round((r(i)-rmin)/0.000563))/1000 * ...
                (r(i)-r(i+1))/0.0563*45*pi/180); 
        else 
            EtransformCumul = EtransformCumul + ...
                (torque_tr_co(1)/1000 * (r(i)-r(i+1))/rmin*45*pi/180);
%             disp('its 0');
        end
    else
        EtransformCumul = EtransformCumul+ (0);
    end

%% velocity contorl
%     omega_d(i+1) = velocity_d(i+1) / r(i+1); 
    Mot_des_dq(i+1) = velocity_d(i+1) / r(i+1) / gear_ratio * rad2rpm;
    
    J_loads = 0.5 * m * r(i+1)^2;
    rs = r(i+1);
    [time, Y] = ode45(@wheel_ode, tspan, X(i,:), options);
    n = length(time);
    X(i+1,:) = Y(n,:); 
    
    Wheel_q(i+1) = mod(X(i+1,1),2*pi); Wheel_dq(i+1) = X(i+1,2); Wheel_ddq(i+1) = ddqpass;
    Mot_dq(i+1) = Wheel_dq(i+1) / gear_ratio * rad2rpm; 
    Mot_ddq(i+1) = Wheel_ddq(i+1) / gear_ratio * rad2rpm; 
    
    Mot_pid_dr(i+1,:) = f_pid([0.1,0.07,0.0001, Mot_des_dq(i+1), Mot_dq(i+1), dt, ...
        Mot_pid_dr(i,1), Mot_pid_dr(i,2), Mot_pid_dr(i,3)]);
    
%     if Mot_pid_dr(i+1,3) > (Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio) %max static force
%         Mot_pid_dr(i+1,3) = Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio;
%     elseif Mot_pid_dr(i+1,3) < -(Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio)
%         Mot_pid_dr(i+1,3) = -(Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio); 
%     end
    if abs(Mot_pid_dr(i+1,3)) > (Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio)
        Mot_pid_dr(i+1,3) = sign(Mot_pid_dr(i+1,3)) * ...
            (Max_Force * r(i+1) / 0.7 * 1000 * gear_ratio);
    end
    
%     if Mot_pid_dr(i+1,3) == 0
%         Motor_load(i+1) = Mot_pid_dr(i+1,3);
%     else
%         Motor_load(i+1) = sign(Mot_pid_dr(i+1,3))*(abs(Mot_pid_dr(i+1,3)) + ...
%             tor_constant * no_load_current);
%     end

    Motor_load(i+1) = Mot_pid_dr(i+1,3);

% Mot pid 3 = motor torque
%     if Mot_pid_dr(i+1,3) > 0
%         Motor_load(i+1) = Mot_pid_dr(i+1,3) + tor_constant * no_load_current;
%     elseif Mot_pid_dr(i+1,3) < 0
%         Motor_load(i+1) = Mot_pid_dr(i+1,3) - tor_constant * no_load_current;
%     else
%         Motor_load(i+1) = Mot_pid_dr(i+1,3);
%     end
    
    Tor_dr = Mot_pid_dr(i+1,3) /1000/ gear_ratio * 0.7; % motor side to wheel side 
    
    Actual_Efficiency(i+1) = eff_actual(Mot_dq(i+1)*rpm2rad*r(i+1), r(i+1),Motor_load(i+1));
%     Actual_Efficiency(i+1) = Receive;
    
%     
%     if Mot_ddq(i)>0
%         J_loads = 0.5 * m * r(i+1)^2 - ; 
%     elseif Mot_ddq(i) < 0
%         J_loads = 0.5 * m * r(i+1)^2;
%     end
    dq_global = Mot_dq(i+1);
    ddq_global = Mot_ddq(i+1);
    torques(i+1) = Tor_dr; 
%     M_eff(i+1) = curLoad;
    Load_from_Eff_d(i+1) = Desired_M;
%     Load_from_Eff_currentR(i+1) = curLoad;
    
end

%%Energy Calculation 
Desired_Power = (Load_from_Eff_d/1000*0.7/gear_ratio .* (velocity_d./rd) ./ Desired_Efficiency); 
Desired_Energy = trapz(t, Desired_Power);
Actual_Power = torques .* Wheel_dq ./ Actual_Efficiency; 
Actual_Energy = trapz(t, Actual_Power);

vel_act = Wheel_dq .* r;
figure(1) 
subplot(2,1,1)
plot(t, Mot_des_dq * rpm2rad * gear_ratio);
hold on
plot(t, Wheel_dq);
hold off
subplot(2,1,2)
plot(t, Desired_Efficiency);
hold on
plot(t, Actual_Efficiency);
ylim([-1 1]);
legend('desired eff','actual eff');
hold off
% 
figure(2)
subplot(4,1,1)
plot(t,Load_from_Eff_d,'Linewidth',2)
hold on
plot(t,Motor_load)
% plot(t,M_eff)
legend('Load of Desired Eff','ML')

subplot(4,1,2)
plot(t,rd)
hold on
plot(t,r)
legend('desired r','actual r')

subplot(4,1,3)
plot(t, velocity_d*rad2rpm/gear_ratio./rd)
hold on
plot(t, Mot_dq)
legend('Desired Motor Speed','Actual Motor Speed');

subplot(4,1,4)
plot(t, accel_d*rad2rpm/gear_ratio./rd)
hold on
plot(t, Mot_ddq)
legend('Desired Motor Accel','Actual Motor Accel');

omega = velocity_d*rad2rpm/gear_ratio./r;
alpha = diff(omega)/dt;
alpha(end+1) = 0;

figure(3)
h1 = plot(t,alpha)
hold on
h2 = plot(t,accel_d*rad2rpm/gear_ratio./rd);
h3 = plot(t,Mot_ddq);
% legend('realistic desired alpha','formal desired alpha','actual alpha');
set(gca, 'FontName','Helvetica');
hTitle = title('Acceleration Plot');
hXLabel = xlabel('Time ({\its})');
hYLabel = ylabel('Acceleration ({\itm/s}^2)');
hLegend = legend([h1, h2, h3], 'Realistic Desired alpha','Formal Desired alpha', ...
    'Actual alpha');
set([hTitle, hXLabel, hYLabel], 'FontName', 'AvantGarde')
set([hLegend, gca], 'FontSize', 8);
set([hXLabel, hYLabel], 'FontSize', 10)
set(hTitle, 'FontSize', 12, 'FontWeight' , 'bold')
set(gca, 'Box', 'off', 'TickDir', 'out', 'TickLength', [.02 .02], ...
    'XMinorTick', 'on', 'YMinorTick', 'on', 'YGrid', 'on', ...
    'XColor', [.3 .3 .3], 'YColor', [.3 .3 .3], 'YTick', -5000:500:5000, ...
    'LineWidth', 1)

figure(4)
subplot(3,1,1)
plot(t, velocity_d)
hold on
plot(t, Mot_dq.*rd*rpm2rad*gear_ratio)
xlabel('Time t (s)');
ylabel('Linear Velocity (m/s)');
title('Velocity Graph');
legend('Desired Velocity','Actual Velocity');
subplot(3,1,2)
plot(t,rd*1000);
hold on
plot(t,r*1000);
xlabel('Time t (s)');
ylabel('Wheel Radius (mm)');
title('Radius Graph');
legend('Desired Radius','Actual Radius');
subplot(3,1,3)
plot(t,Desired_Efficiency)
hold on
plot(t,Actual_Efficiency)
ylim([-1 1]);
xlabel('Time t (s)');
ylabel('Motor Efficiency');
title('Efficiency Graph');
legend('Desired Efficiency','Actual Efficiency');

figure(5)
plot(t,Desired_Efficiency)
ylim([-1 1])

% 
% figure(6)
% hold on
% h1 = plot(t,rd*1000);
% h2 = plot(t,rd2*1000);
% h3 = plot(t,rd3*1000);
% h4 = plot(t,rd4*1000);
% h5 = plot(t,rd5*1000);
% hXLabel = xlabel('Time ({\its})');
% hYLabel = ylabel('Wheel Radius ({\itmm})');
% hTitle = title('Desired Radius Graph');
% hLegend = legend([h1,h2,h3,h4,h5],'Incline=3 deg','Incline=6 deg','Incline=9 deg', ...
%     'Incline=12 deg','Incline=15 deg');
% set([hLegend, gca], 'FontSize', 8);
% set([hXLabel, hYLabel], 'FontSize', 11)
% set(hTitle, 'FontSize', 11, 'FontWeight' , 'bold')
% % hLegend = legend([h1, h3, h4],'Incline=4 deg','Incline=6 deg','Incline=8 deg');
toc




