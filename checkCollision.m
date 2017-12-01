function  [r_security]= checkCollision(P_Robot,v_Robot,radius_robot,P_Obs,u_Obs,radius_Obs,gama,theta)
% Collision-cone approach for mobile obstacles
d = sqrt((P_Robot(1)-P_Obs(1))^2 + (P_Robot(2)-P_Obs(2))^2);

% Relative speed
U_r = (u_Obs(1) * cos(P_Obs(3)-theta)) - v_Robot(1) * cos(P_Robot(3)-theta); % 57.2957* ??
U_theta =  (u_Obs(1) * sin(P_Obs(3)-theta)) - v_Robot(1) * sin(P_Robot(3)-theta); % 57.2957* ??

h = abs(U_theta)/(sqrt(U_theta^2 + U_r^2));

r_double = min(radius_Obs*gama*sigmoid(1,2,h),d-radius_Obs-radius_robot);

if U_r<0
    r_security = radius_Obs + radius_robot + r_double;
else
    r_security = radius_Obs + radius_robot;
end

end
