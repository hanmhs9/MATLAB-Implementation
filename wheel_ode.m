function dX=wheel_ode(t,X)
%% Set the variable
global gear_ratio 
global GradAngle GradAngleD
global Tor_dr m  mur rs ddqpass loadpass J_motor J_loads testLoad
q_ws=X(1);
dq_ws=X(2);
xDis = X(3);
yDis = X(4);
% Km = 19.9; %mNm/A
% Io = 0.479; %A
g = 9.81;
MR = 19.9 * 0.213; 
MRW = MR / 0.7 / gear_ratio / 1000;


Jtotal = J_loads+J_motor/(gear_ratio^2) + 859.714*10^-9*(43/30)^2;

% Aerodynamic Drag Resistance
rho = 1.225; %kg/m^3
As = (0.27*0.19) + 2*(2*0.1*0.045);
Cd = 0.5; %drag coeff
Fds = 0.5*rho * sign(dq_ws)*(dq_ws*rs)^2 * Cd * As;
% Fds = (dq_ws * rs)^2 * 0.426;
Td = Fds * (0.19/2 + rs);

% GradAngle = 0.1;
% GradAngleD = GradAngle + asind((rs-0.05)/0.4);
Tg = m*g*sind(GradAngle) * rs;

%rolling resistance and static friction
Tr = m*g*cosd(GradAngle)*mur*rs * sign(dq_ws); % rolling resistance

% netT = Tor_dr - Tr - Td - Tg;
netT = Tor_dr - Tr - Td - Tg - sign(dq_ws)*MRW;
% Tor_dr
% Motor_load = Tor_dr*1000*gear_ratio / 0.7
% netT
% testLoad
I = Jtotal;
ddq_ws = netT/(I+m*rs^2);
% ddq_ws = (MRW<abs(netT))*(netT-sign(netT)*MRW)/(I+m*rs^2);
totalInertia = I+m*rs^2;
ddqpass = ddq_ws;
% loadpass = Tr + Td + Tg;

%acceleration
a = ddq_ws * rs; 


%Body velocity
vx = dq_ws * rs * cosd(GradAngle);
vy = dq_ws * rs * sind(GradAngle); 




%% Set the result
dX(1)=dq_ws;
dX(2)=ddq_ws;
dX(3)= vx;
dX(4)= vy;
dX=dX';


end