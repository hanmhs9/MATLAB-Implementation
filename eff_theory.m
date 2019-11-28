function efficiency = eff_theory(ve, angle)
global Tor_dr GradAngle GradAngleD curLoad Desired_M mur gear_ratio Current_Radius_and_Efficiency J_motor m maxLoad minLoad eff_to_main Im_Eff_Rmin Im_Eff_Rmax
global dq_global ddq_global v_prev rs GradAngle2 GradAngle3 GradAngle4 rs2 rs3 rs4
global GradAngle5 rs5
%% necessary basic variables
% R = 0.582; %ohm 
% Km = 29.2; %mNm/A
% Kn = 328;  %rpm/V
% Io = 0.128; %A
% no = 7750;
% 
R = 0.358; %ohm
Km = 19.9; %mNm/A
Kn = 479;  %rpm/V
Io = 0.213; %A
no = 8590; %rpm
MR = Io*Km;
MRW = MR / 0.7 / gear_ratio / 1000;

m1 = 1.5; %kg
dt = 0.01;
mur = 0.01;  %coefficient
g = 9.81; % m/s^2
rho = 1.225; %kg/m^3
As = (0.27*0.19) + 2*(2*0.1*0.045); %Cross sectional area, m^2
Cd = 0.5; %drag coeff
rpm2rad = pi/30;
rad2rpm = 30/pi;


%% Efficiency Formula
% a = ac; %a and v is already increased by the amount of the gear_ratio
v = ve;
theta = angle * pi/180;
theta2 = GradAngle2 * pi/180;
theta3 = GradAngle3 * pi/180;
theta4 = GradAngle4 * pi/180;
theta5 = GradAngle5 * pi/180;

r = [0.06: 0.001: 0.113];

Fds = 0.5*rho * sign(v)*(v*gear_ratio).^2 * Cd * As;
Td = Fds .* (0.19/2 + r); % 영향거의 없음
Tr = m*g*r*cos(theta)*mur*sign(v);
Tr2 = m*g*r*cos(theta2)*mur*sign(v);
Tr3 = m*g*r*cos(theta3)*mur*sign(v);
Tr4 = m*g*r*cos(theta4)*mur*sign(v);
Tr5 = m*g*r*cos(theta5)*mur*sign(v);

Tg = m*g*sin(theta)*r;
Tg2 = m*g*sin(theta2)*r;
Tg3 = m*g*sin(theta3)*r;
Tg4 = m*g*sin(theta4)*r;
Tg5 = m*g*sin(theta5)*r;

%% Locating M and Efficiencies 
a = (v-v_prev) / dt;
v_prev = v; 

% a = ddq_global * rs * rpm2rad;
M = (Tr + Tg + Td + MRW*sign(v) + a.*gear_ratio./r.*(0.5*m.*r.^2 + ...
        J_motor./(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m.*r.^2)) ... 
        *gear_ratio / 0.7 *1000;
% M2 = (Tr2 + Tg2 + Td + MRW*sign(v) + a.*gear_ratio./r.*(0.5*m.*r.^2 + ...
%         J_motor./(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m.*r.^2)) ... 
%         *gear_ratio / 0.7 *1000;
% M3 = (Tr3 + Tg3 + Td + MRW*sign(v) + a.*gear_ratio./r.*(0.5*m.*r.^2 + ...
%         J_motor./(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m.*r.^2)) ... 
%         *gear_ratio / 0.7 *1000;
% M4 = (Tr4 + Tg4 + Td + MRW*sign(v) + a.*gear_ratio./r.*(0.5*m.*r.^2 + ...
%         J_motor./(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m.*r.^2)) ... 
%         *gear_ratio / 0.7 *1000;
% M5 = (Tr5 + Tg5 + Td + MRW*sign(v) + a.*gear_ratio./r.*(0.5*m.*r.^2 + ...
%         J_motor./(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m.*r.^2)) ... 
%         *gear_ratio / 0.7 *1000;
    
for ii=1:length(M)
    if any(abs(M(ii)) < MR || v == 0)
        eff(ii) = 0;
    else
        eff(ii) = sign(M(ii)*v)*1./(1+sign(M(ii)*v)* ... 
            (30000/pi*R*M(ii)./(v./r(ii)*30/pi)/Km^2)) ...
        - 1./(sign(M(ii)*v)*abs(M(ii))/MR + ...
        sign(M(ii)*v)* 30000/pi * R*(M(ii).^2)./ ...
        (abs(v)./r(ii)*30/pi)/MR/Km^2);
    end
%     if any(abs(M2(ii)) < MR || v == 0)
%         eff2(ii) = 0;
%     else
%         eff2(ii) = sign(M2(ii)*v)*1./(1+sign(M2(ii)*v)* ... 
%             (30000/pi*R*M2(ii)./(v./r(ii)*30/pi)/Km^2)) ...
%         - 1./(sign(M2(ii)*v)*abs(M2(ii))/MR + ...
%         sign(M2(ii)*v)* 30000/pi * R*(M2(ii).^2)./ ...
%         (abs(v)./r(ii)*30/pi)/MR/Km^2);
%     end
%     if any(abs(M3(ii)) < MR || v == 0)
%         eff3(ii) = 0;
%     else
%         eff3(ii) = sign(M3(ii)*v)*1./(1+sign(M3(ii)*v)* ... 
%             (30000/pi*R*M3(ii)./(v./r(ii)*30/pi)/Km^2)) ...
%         - 1./(sign(M3(ii)*v)*abs(M3(ii))/MR + ...
%         sign(M3(ii)*v)* 30000/pi * R*(M3(ii).^2)./ ...
%         (abs(v)./r(ii)*30/pi)/MR/Km^2);
%     end
%     if any(abs(M4(ii)) < MR || v == 0)
%         eff4(ii) = 0;
%     else
%         eff4(ii) = sign(M4(ii)*v)*1./(1+sign(M4(ii)*v)* ... 
%             (30000/pi*R*M4(ii)./(v./r(ii)*30/pi)/Km^2)) ...
%         - 1./(sign(M4(ii)*v)*abs(M4(ii))/MR + ...
%         sign(M4(ii)*v)* 30000/pi * R*(M4(ii).^2)./ ...
%         (abs(v)./r(ii)*30/pi)/MR/Km^2);
%     end
% 
%     if any(abs(M5(ii)) < MR || v == 0)
%         eff5(ii) = 0;
%     else
%         eff5(ii) = sign(M5(ii)*v)*1./(1+sign(M5(ii)*v)* ... 
%             (30000/pi*R*M5(ii)./(v./r(ii)*30/pi)/Km^2)) ...
%         - 1./(sign(M5(ii)*v)*abs(M5(ii))/MR + ...
%         sign(M5(ii)*v)* 30000/pi * R*(M5(ii).^2)./ ...
%         (abs(v)./r(ii)*30/pi)/MR/Km^2);
%     end
end
        
% M = M + sign(M(1))*MRW*gear_ratio / 0.7 *1000;
% for ii=1:length(M)
%     if M(ii) > 0 
%         M(ii) = M(ii) + MR;
%         eff(ii) = 1./(1+30000/pi*R*M(ii)./(v./r(ii)*30/pi)/Km^2) ...
%             - 1./(M(ii)/MR + 30000/pi * R*(M(ii).^2)./(v./r(ii)*30/pi)/MR/Km^2);
%     elseif M(ii) < 0 
%         M(ii) = M(ii) - MR;
%         eff(ii) = 1./(1-30000/pi*R*M(ii)./(v./r(ii)*30/pi)/Km^2) ...
%             + 1./(M(ii)/MR - 30000/pi * R*(M(ii).^2)./(v./r(ii)*30/pi)/MR/Km^2);
%     elseif M(ii) == 0 
%         M(ii) = M(ii);
%         eff(ii) = 0;
%     end
% end

% M = M + sign(M)*MR;


% eff = sign(M(1)*v(1))*1./(1+sign(M(1)*v(1))*(30000/pi*R*M./(v./r*30/pi)/Km^2)) ...
%         - 1./(sign(M(1)*v(1))*abs(M)/MR + sign(M(1)*v(1))* 30000/pi * R*(M.^2)./ ...
%         (abs(v)./r*30/pi)/MR/Km^2);

% if M > 0
%     M = M + MR;
%     eff = 1./(1+30000/pi*R*M./(v./r*30/pi)/Km^2) ...
%             - 1./(M/MR + 30000/pi * R*(M.^2)./(v./r*30/pi)/MR/Km^2);
% elseif M < 0 
%     M = M - MR;
% %     eff = 1./(1-30000/pi*R*M./(v./r*30/pi)/Km^2) ...
% %             + 1./(M/MR - 30000/pi * R*(M.^2)./(v./r*30/pi)/MR/Km^2);
%     eff = - 1./(1-30000/pi*R*M./(v./r*30/pi)/Km^2) ...
%             - 1./(M/MR - 30000/pi * R*(M.^2)./(v./r*30/pi)/MR/Km^2);   
% else
%     M = M;
%     eff(1:length(M)) = 0;
% end
% maxEfficiency = max(abs(eff)); %Maximum Efficiency 
% maxEfficiency = (-min(eff) < max(eff))*(max(eff)-min(eff))+min(eff);
[~,ii] = max(abs(eff));
maxEfficiency = eff(ii);

% [~,jj] = max(abs(eff2));
% maxEfficiency2 = eff2(jj);
% 
% [~,kk] = max(abs(eff3));
% maxEfficiency3 = eff3(kk);
% 
% [~,ll] = max(abs(eff4));
% maxEfficiency4 = eff4(ll);
% 
% [~,mm] = max(abs(eff5));
% maxEfficiency5 = eff5(mm);

% minEfficiency = min(abs(eff)); %Minimum Efficiency
    
%% Efficiency of the Current Radius

%% Ideal Radius of the Maximum Efficiency
% if GradAngle >= 0
% [value rcIndex] = min(abs(r-rc))
if maxEfficiency ~= 0
    Index = find(eff==maxEfficiency);
    theRadius = r(Index);
else
    [f Index] = min(abs(r-rs));
%     Index = find(r==rs);
    theRadius = r(Index);
end
% 
% if maxEfficiency2 ~= 0
%     Index2 = find(eff2==maxEfficiency2);
%     rs2 = r(Index2);
% else
%     [f Index2] = min(abs(r-rs2));
% %     Index = find(r==rs);
%     rs2 = r(Index2);
% end
% 
% if maxEfficiency3 ~= 0
%     Index3 = find(eff3==maxEfficiency3);
%     rs3 = r(Index3);
% else
%     [f Index3] = min(abs(r-rs3));
% %     Index = find(r==rs);
%     rs3 = r(Index3);
% end
% 
% if maxEfficiency4 ~= 0
%     Index4 = find(eff4==maxEfficiency4);
%     rs4 = r(Index4);
% else
%     [f Index4] = min(abs(r-rs4));
% %     Index = find(r==rs);
%     rs4 = r(Index4);
% end
% 
% if maxEfficiency5 ~= 0
%     Index5 = find(eff5==maxEfficiency5);
%     rs5 = r(Index5);
% else
%     [f Index5] = min(abs(r-rs5));
% %     Index = find(r==rs);
%     rs5 = r(Index5);
% end

%     rs
efficiency = [theRadius, maxEfficiency];
%     efficiency = [theRadius, maxEfficiency, rs2, maxEfficiency2, rs3, ...
%         maxEfficiency3, rs4, maxEfficiency4, rs5, maxEfficiency5]; % Return
    
%% Desired Load of Motor

%MR = 4.2387
% Desired_M = (mur*m*g*theRadius*cos(theta)*sign(v) + ...
%     m*g*theRadius*sin(theta)+ Td(Index) + a*gear_ratio/theRadius*(0.5*m*theRadius^2 ...
%     + J_motor/(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m*theRadius^2))/0.7*gear_ratio*1000;
% if Desired_M > 0
%     Desired_M = Desired_M + MR;
% elseif Desired_M < 0
%     Desired_M = Desired_M - MR;
% else
%     Desired_M = 0;
% end
Desired_M = M(Index);

% if M >= 0
%     Desired_M = (mur*m*g*theRadius*cos(theta) + m*g*theRadius*sin(theta)+ Td(Index) + a*gear_ratio/theRadius*(0.5*m*theRadius^2 ...
%      + J_motor/(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m*theRadius^2))/0.7*gear_ratio*1000 + Io*Km;
% elseif M < 0
%     Desired_M = (mur*m*g*theRadius*cos(theta) + m*g*theRadius*sin(theta)+ Td(Index) + a*gear_ratio/theRadius*(0.5*m*theRadius^2 ...
%      + J_motor/(gear_ratio^2) + 859.714*10^-9*(43/30)^2 + m*theRadius^2))/0.7*gear_ratio*1000 - Io*Km;
% end 


end