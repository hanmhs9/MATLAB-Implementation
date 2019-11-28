function efficiency = eff_actual(v,r,M)

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
MR = Km * Io;
if abs(M) > MR
    efficiency = sign(M*v)*1/(1+sign(M*v)*(30000/pi*R*M/(v/r*30/pi)/Km^2)) ...
        - 1/(sign(M*v)*abs(M)/MR + sign(M*v)* 30000/pi * R*(M^2)/(abs(v)/r*30/pi)/MR/Km^2);
else
    efficiency = 0;
end 

% if M > MR
% %     M = M + Io*Km; %already accounted in main
% %     efficiency = (1-((Kn*Km^2*pi*r/(v*30*Km^2+30000*R*M*r))*(M*R/Km-Io*R)+Io*Km/M));
%     efficiency = 1/(1+30000/pi*R*M/(v/r*30/pi)/Km^2) - 1/(M/MR + 30000/pi * R*(M^2)/(v/r*30/pi)/MR/Km^2); 
% elseif M < -MR
% %     M = M - Io*Km; %already accounted in main
% %     efficiency = (1+((Kn*Km^2*pi*r/(v*30*Km^2+30000*R*M*r))*(M*R/Km-Io*R)+Io*Km/M));
%     efficiency = - 1/(1-30000/pi*R*M/(v/r*30/pi)/Km^2) - 1/(M/MR - 30000/pi * R*(M^2)/(v/r*30/pi)/MR/Km^2);
% else
%     efficiency = 0;  
%      
% end

% figure(5) 
% M = 1:3:1000;
% % efficiency = (1-((Kn*Km^2*pi*r./(v*30*Km^2+30000*R.*M*r)) .* (M*R./Km-Io*R) + Io*Km./M)); 
% efficiency = 1./(1+30000/pi*R.*M/(v/r*30/pi)/Km^2) - 1./(M./MR + 30000/pi * R*(M.^2)/(v/r*30/pi)/MR/Km^2);
% plot(M,efficiency);
% max(efficiency)
% mean(efficiency)
% figure(6)
% M = -1000:3:-1;
% % efficiency = (1+((Kn*Km^2*pi*r./(v*30*Km^2+30000*R.*M*r)).*(M*R./Km-Io*R) + Io*Km./M));
% efficiency =  1./(1-30000/pi*R*M/(v/r*30/pi)/Km^2) + 1./(M/MR - 30000/pi * R*(M.^2)/(v/r*30/pi)/MR/Km^2);
% plot(M,efficiency);
% max(efficiency)
% mean(efficiency)
end