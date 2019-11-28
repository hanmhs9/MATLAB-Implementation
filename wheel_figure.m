function wheel_figure(q, dq, t, dx, dy, angle, radius)
%  wheel_figure(Wheel_q(i+1),Wheel_dq(i+1),t(i+1), xDis(i+1), yDis(i+1), gradi_Angle(i+1), r(i+1));
global py scale px 
q_wheel = q;

dq_wheel = dq;
rs = radius;
scale = 40;
q_pwheel = mod(rs/0.05 * q,2*pi);
dq_pwheel = rs/0.05 * dq;

r_wheel = rs * scale;
passive_wheel = 0.05 * scale;
xpos = dx * scale;
% ypos = dy * scale;

d_g_i = 0.4 * scale; %12.8
gAngle = angle;
bAngle = angle + asind((rs-0.05)/0.4);
bodyh=0.19*scale;
trrh = py * scale; 
trrl = px * scale; 
if gAngle>=0
    road = 0;
else 
    road = 5;
      
end

gap = d_g_i/2 + 0.2; %6.4+0.2 

[ho gapindex] = min(abs(trrl-(trrl(1)+gap)));
for i =1:360
    wheel(i,:) = [r_wheel*cosd(i), r_wheel*sind(i)];
    p_wheel(i,:) = [passive_wheel*cosd(i), passive_wheel*sind(i)];
    dot(i,:) = [0.1*cosd(i), 0.1*sind(i)];
end

curmin = 1;
[c ind] = min(abs(trrl-xpos));
index = ind;


if gAngle > 0
    mea = trrh(index - 2*gapindex) + 4.5;
    tol = -4.5;
    toRed = trrh(index-2*gapindex) - tol; 
elseif gAngle < 0 
    mea = trrh(index + 3*gapindex) + 4.5;
    tol = -4.5;
    toRed = trrh(index+3*gapindex) - tol; 
else
    mea = trrh(index - 2*gapindex) + 4.5;
    tol = -4.5;
    toRed = trrh(index-2*gapindex) - tol; 
end


%% wheel bar 
coor_wheel1 = [0 0;0 0.5; r_wheel 0.5;r_wheel 0]; %bar 
coor_wheel2 = [0 0;0 0.2; passive_wheel 0.2; passive_wheel 0];

%% body points 
pos1 = [-passive_wheel*sind(gAngle) passive_wheel*cosd(gAngle)+road]; %bottom left 
pos4 = [pos1(1)+2*gap*cosd(bAngle) pos1(2)+2*gap*sind(bAngle)]; %bottom right
pos2 = [pos1(1)-0.5*cosd(bAngle) pos1(2)+0.5*sind(bAngle)]; 
pos3 = [pos2(1)-2*sind(bAngle) pos2(2)+2*cosd(bAngle)];
pos4_2 = [pos3(1)+gap/1.5*cosd(bAngle) pos3(2)+gap/1.5*sind(bAngle)];
pos5 = [pos4_2(1)+1.7*sind(bAngle) pos4_2(2)-1.7*cosd(bAngle)];
pos6 = [pos5(1)+(gap-(pos5(1)-pos1(1)))*cosd(bAngle) pos5(2)+(gap-(pos5(1)-pos1(1)))*sind(bAngle)];
pos7 = [pos6(1)+(gap-(pos5(1)-pos1(1)))*cosd(bAngle) pos6(2)+(gap-(pos5(1)-pos1(1)))*sind(bAngle) ];
% pos3_2 = [pos2(1)+2*gap
pos8 = [pos7(1)-2*sind(bAngle) pos7(2)+2*cosd(bAngle)];
pos9 = [pos8(1)+gap/2*cosd(bAngle) pos8(2)+gap/2*sind(bAngle)];
pos10 = [pos9(1)+2.2*sind(bAngle) pos9(2)-2.2*cosd(bAngle)];
% pos11 = [pos10(1)-1.8*sind(bAngle) pos10(2)+1.8*cosd(bAngle)];
% pos12 = [pos11(1)-4*cosd(bAngle) pos11(2)-4*sind(bAngle)];
% pos13 = [pos12(1)+1.8*sind(bAngle) pos12(2)-1.8*cosd(bAngle)];
pos11 = [pos10(1)-1*cosd(bAngle) pos10(2)+1*cosd(bAngle)];
pos12 = [pos11(1)-1.5*cosd(bAngle) pos11(2)+1.5*sind(bAngle)];
pos13 = [pos12(1)-5.0*cosd(bAngle) pos12(2)-5*sind(bAngle)];
pos14 = [pos13(1)-1.0*cosd(bAngle) pos13(2)-1*sind(bAngle)];
pos15 = [pos10(1)-10*cosd(bAngle) pos10(2)-10*sind(bAngle)];




%% body position
coor_body = [pos1;pos4; pos10; pos2];

% coor_body2 = [pos1; pos2; pos5; pos6; pos7; pos8; pos9; pos10];
coor_body2 = [pos10; pos11; pos12; pos13; pos14; pos15];

% batpo1 = [pos1(1)+5*cosd(gAngle) pos1(2)+5*sind(gAngle)];
% batpo2 = [pos4_2(1)+1.5*cosd(gAngle) pos2_2(2)+2*sind(gAngle)];
% batpo3 = [batpo2(1)+3*cosd(gAngle) batpo2(2)+3*sind(gAngle)];
% batpo4 = [batpo1(1)*2*cosd(gAngle) batpo1(2)+4*sind(gAngle)];
% coor_bat = [batpo1; batpo2; batpo3; batpo4];
%% grid (plot) position
axislength = 41;
axisheight = 18;
coor_grid1 = [39 23; 39 23+axisheight; 39+axislength 23+axisheight; 39+axislength 23];
coor_grid2 = [39 -1; 39 -1+axisheight; 39+axislength -1+axisheight; 39+axislength -1];
coor_grid3 = [39 -25; 39 -25+axisheight; 39+axislength -25+axisheight; 39+axislength -25];



coor_wheel1 = rotation([coor_wheel1'], -q_wheel);
coor_wheel2 = rotation([coor_wheel2'], -q_wheel);


coor_wheel1 = coor_wheel1';
coor_wheel2 = coor_wheel2';



figure(1)
set(gcf, 'Position', [100, 350, 800, 600])
fill(coor_body(:,1),coor_body(:,2),'w');

hold on

fill(coor_body2(:,1),coor_body2(:,2),'w');


%terrain
if mea>tol || mea < tol && gAngle >= 0
    toRed = trrh(index-2*gapindex) - tol;         
elseif gAngle < 0
    toRed = trrh(index+3*gapindex) - tol;
else
    toRed = 0;
end

%% road
line([-gap*cosd(gAngle) 0],[-gap*sind(gAngle)+road 0+road],'Color','black','Linewidth',3);
line([0 gap*cosd(gAngle)],[0+road gap*sind(gAngle)+road],'Color','black','Linewidth',3);
line([gap*cosd(gAngle) 2*gap*cosd(gAngle)],[gap*sind(gAngle)+road 2*gap*sind(gAngle)+road],'Color','black','Linewidth',3);
line([2*gap*cosd(gAngle) 3*gap*cosd(gAngle)],[2*gap*sind(gAngle)+road 3*gap*sind(gAngle)+road],'Color','black','Linewidth',3);
line([3*gap*cosd(gAngle) 4*gap*cosd(gAngle)],[3*gap*sind(gAngle)+road 4*gap*sind(gAngle)+road],'Color','black','Linewidth',3);
% gAngle


%left wheel
fill(pos1(1)+p_wheel(:,1), pos1(2)+p_wheel(:,2),'w');
%right wheel
fill(pos4(1)+wheel(:,1),pos4(2)+wheel(:,2),'w'); %right wheel start position overwrite


%bars
fill(pos1(1)+2*gap*cosd(bAngle)+coor_wheel1(:,1), pos1(2)+2*gap*sind(bAngle)+coor_wheel1(:,2),'k');
fill(pos1(1)+coor_wheel2(:,1),pos1(2)+coor_wheel2(:,2),'k');


axis equal
xlim([-7 80])
axis off
text(-20+d_g_i/2,-10,'Wheel Speed','Fontsize',13)
if dq_wheel < 0.0001
    dq_wheel = 0;
end
vel_wheel = strcat(num2str(round(dq_wheel,3,'significant')));
text(-20+d_g_i/2,-14,vel_wheel,'Fontsize',13)
text(-20+d_g_i/2 + length(vel_wheel)+1,-14,' rad/s','Fontsize',13)

time = strcat(num2str(round(t,4,'significant')));
text(0,30,'Time:','Fontsize',13)
text(9,30,time,'Fontsize',13)
text(9+length(time)+1,30,' s','Fontsize',13)

rshow = strcat(num2str(round(rs*1000*2,3,'significant')));
text(10,-10,'Wheel Size','Fontsize',12);
text(10,-14, rshow,'Fontsize',12);
text(10+length(rshow)+1,-14,' mm','Fontsize',12);


text(50, 44, 'Velocity vs Time','Fontsize',13);
fill(coor_grid1(:,1),coor_grid1(:,2),'w');

text(50, 20, 'Power vs Time','Fontsize',13);
fill(coor_grid2(:,1),coor_grid2(:,2),'w');

text(50, -4, 'Efficiency vs Time','Fontsize',13);
fill(coor_grid3(:,1),coor_grid3(:,2),'w');
t
grid(t,d_g_i, rs, dx,dy);
    pause(0.001);
    hold off
    
 

end
function R = rotation(P,angle)
R = [cos(angle) -sin(angle);sin(angle) cos(angle)]*P;
end
function G = grid(t,d_g_i, radi, x,y)
    global passtime passPower passEff passdq passveld rglobal EtransformPass
    dx = x;
    dy = y;
    rs = radi;   
    axislength = 41;
    axisheight = 15;
    
    ind = find(passtime==t);
    length(passPower);
    p1 = plot(passtime/(max(passtime)/axislength)+39.1, passveld/((max(passdq.*rglobal')-min(passdq.*rglobal'))/15)+24,'Linewidth',1);
    p2 = plot(passtime(1:ind)/(max(passtime)/axislength)+39.1, passdq(1:ind).*rglobal(1:ind)'/((max(passdq.*rglobal')-min(passdq.*rglobal'))/15)+24,'r','Linewidth',1.4);

    if passdq(ind) < 0.001
        passdq(ind) = 0;
    end
    velDisp = strcat(num2str(round(passdq(ind)*rs,3,'significant')),' m/s');    
    text(32,passdq(ind)*rs/(max(passveld)/19)+25, velDisp);

    Pdisp = strcat(num2str(round(passPower(ind),3,'significant')),'W');

    p3 = plot(passtime/(max(passtime)/axislength)+39, passPower/(max(passPower)/(axisheight))+1,'r');
    text(32,passPower(ind)/(max(passPower)/19)-1,Pdisp);
    
    Edisp = strcat(num2str(round(passEff(ind)*100,3,'significant')),'%');
    p4 = plot(passtime(1:ind)/(max(passtime)/axislength)+39, passEff(1:ind)/(max(passEff)/axisheight)-24);
    text(32,passEff(ind)/(max(passEff)/axisheight)-24,Edisp);
 
    text(-20+d_g_i/2,-20,'Energy Consumed','Fontsize',12)

    Energy = trapz(passtime(1:ind), passPower(1:ind)) + EtransformPass;
        legend([p1 p2],{'desired','actual','',''}, 'Location', [0.5688 0.8734 0.1178 0.01]);
    legend('boxoff');
    EnergyDisp = strcat(num2str(round(Energy,5,'significant')));
    text(-20+d_g_i/2,-24,EnergyDisp,'Fontsize',12)
    text(-20+d_g_i/2+length(EnergyDisp)+1,-24, ' J', 'Fontsize', 12)
    
    text(10, -20, 'Distance Travelled', 'Fontsize',12)
    Dist = sqrt(dx^2+dy^2); 
    DistDisp = strcat(num2str(round(Dist,3,'significant')));
    text(10,-24,DistDisp,'Fontsize',12)
    text(10+length(DistDisp)+1,-24, 'm','Fontsize',12)
    
end