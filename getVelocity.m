function velocity = getVelocity(num)
    if num==1 % WLTP
        
        profile = getWLTP(); %return [time(s) velocity(m/s) velocity_for_small_wheel(m/s)] 
        dt = 0.01; 
        t = profile(:,1);
        velocity_d = profile(:,2); 
        accel_d = diff(velocity_d)/dt; 
        accel_d(end+1) = 0; %acceleration array is shorter than the velocity array 
        total_time = t(end); 
    
    elseif num==2
        total_time = 30;
        reachtime = total_time/3; %time to reach the peak velocity 
        reachvel = 0.6; %peak velocity to reach 
        dt = 0.01;
        t = 0:dt:total_time;
        [toFind Index1] = min(abs(t-reachtime)); 
%         Index1
        [toFind Index2] = min(abs(t-(total_time-reachtime))); % last 25%
%         Index2
        accel = reachvel/reachtime * t(1:Index1);
        decel = -reachvel/reachtime*t(Index2:length(t));  
        velocity_d = accel;
%         length(velocity_d)
%         length(accel)
        velocity_d(Index1+1:Index2) = reachvel;
        velocity_d(Index2+1:length(t)) = decel;
        accel_d = diff(velocity_d)/dt; 
        accel_d(end+1) = 0; 
    elseif num==3 
        reachtime = 2.5;
        reachvel =0.7;
        dt = 0.01;
        t = 0:dt:10;
        accel = reachvel/t(round(length(t)/2));
        decel = -accel;
        velocity_d = [accel decel];
        accel_d = [diff(velocity_d)/dt 0]; 
    end

    velocity = [velocity_d t accel_d];
end