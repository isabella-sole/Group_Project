
% Looking from the data, I assumed the lidar axes as follows:
% Y positive toward the front of the vehicle
% X positive toward the right-hand side of the vehicle.
% Z positive upwards
% 
% The catamaran has�
% X positive forward
% Y positive to the right
% Z positive down
% 
% The NED frame is
% X positive north
% Y positive east
% Z downward
% 
% The telemetry of the vehicle gives the roll pitch yaw angles of the catamaran wrt the NED frame.
% UTM is X positive east and Y positive north, hence the swap.

A = readmatrix('log_track.txt');
% pred_x, pred_y, pred_vx, pred_vy, est_x, est_y, est_vx,est_vy,cat_latitude,cat_longitude,cat_altitude, cat_speed[0],cat_speed[1]
% all obstcal data are relative to lidar frame
 B = readmatrix('log_pos.txt');
pred_x=A(:,1);
pred_y=A(:,2);
pred_vx=A(:,3);
pred_vy=A(:,4);
est_x=B(:,1);
est_y=B(:,2);
est_vx=A(:,7);
est_vy=A(:,8);
W=A(:,17);
L=A(:,18);
 
% all catamarn data
cat_latitude=A(:,9);
cat_longitude=A(:,10);
cat_altitude=A(:,11);
cat_speed_0=A(:,12);
cat_speed_1=A(:,13);
cat_yaw=A(:,14);
cat_pitch=A(:,15);
cat_roll=A(:,16);
  
% gps to utm conversion
[xUTM,yUTM,utmzone] = deg2utm(cat_latitude,cat_longitude);

xNED = yUTM - yUTM(1);
yNED = xUTM - xUTM(1);
b = [0 0.1];
a = [1 -0.900];
yaw_filtered = filtfilt(b,a,cat_yaw);

bodyRlidar = eul2rotm([ pi/2 0   pi]);
jumps_x = [];
jumps_y = [];

for i=1:length(pred_x)
    worldRbody = eul2rotm([ yaw_filtered(i) cat_pitch(i) cat_roll(i)  ]);
    %worldRbody = rotation(0, 0, cat_yaw(i));
    worldRlidar = worldRbody * bodyRlidar;
    
    catamaran_position(i,:) = [xNED(i), yNED(i), 0];
    obstacle_position(i,:) =  (catamaran_position(i,:)' + worldRlidar * [est_x(i); est_y(i); 0])';
    
      if (i > 1 && norm(obstacle_position(i,:) - obstacle_position(i-1,:)) > 3)
        jumps_x = [jumps_x [i; i]];
        jumps_y = [jumps_y [-10; 10]];
    end
    
    if (i > 1)
        cat_delta(i-1) = norm(catamaran_position(i,:) - catamaran_position(i-1,:));
    end
end

N = length(obstacle_position);
figure
hold off
plot(catamaran_position(:,2),catamaran_position(:,1),'b')
hold on
plot(obstacle_position(:,2),obstacle_position(:,1),'-*r')
axis equal
text(obstacle_position(1:10:N,2), obstacle_position(1:10:N,1), num2str([1:10:N]'))
text(catamaran_position(1:20:N,2), catamaran_position(1:20:N,1), num2str([1:20:N]'))
legend('catamaran', 'obstacle');

figure
plot([cat_roll cat_pitch cat_yaw].*180/pi)
legend('roll', 'pitch', 'yaw');


figure(5); hold off;
plot(est_vx);


figure(6); hold off;
plot(est_vy);

f1=figure(7);hold off;

plot(W);
title('Width in meter')
f2=figure(8);hold off;

plot(L);
title('Length in meter')
