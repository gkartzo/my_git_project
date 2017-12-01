close
GoalPoint = [15 -4]; % Define Goal Point
umax = 2;
gama = 2;
num_obstacles = 2;
ro = 2; % maximum range in whicj robot can scan the workspace
% sin, cos uses rad
% Time (sec)
dt = 1;
maxtime = 60;
time = 0:dt:maxtime;


% Obstacle 1
radius_Obs1 = 0.5;
P_Obs1 = zeros(length(time),3);
crc = @(c,r,p,a) [c(1) + r*cosd(a + p); c(2) +  r*sind(a + p)];
init_pos = -90; % Initial position (�)
radius = 6;
velocity1 = 13; % Constant rotational velocity 
center = [0 0];
locus = crc(center,radius, init_pos, -velocity1*time);
for t = 1:length(time)
    P_Obs1(t,1) = locus(1,t);
    P_Obs1(t,2) = locus(2,t);
    P_Obs1(t,3) = FindHeadingAngle(P_Obs1(t,:),center); % Inverse tangent in degrees (�)  
end

% % Obstacle 2
% radius_Obs2 = 0.5;
% P_Obs2 = zeros(length(time),3);
% velocity2 = 0.75; % Constant longitudinal velocity
% P_Obs2(1,:) = [-14 -4 0]; % Initial position
% for i = 2:length(time)
%     P_Obs2(i,1) = P_Obs2(i-1,1) + velocity2*dt;    % Update position
%     P_Obs2(i,2) = P_Obs2(i-1,2) + velocity2*dt; 
%     P_Obs2(i,3) = atand((P_Obs2(i,2) - P_Obs2(i-1,2))/((P_Obs2(i,1) - P_Obs2(i-1,1)))); % The heading angle of the vehicle
% end

radius_Obs_all = [radius_Obs1; radius_Obs2];
%%

% Obstacle 3
u_ob3 = 2;
direction_coef = 0.8;

radius_Obs2 = 0.5;
P_Obs2 = zeros(length(time),3);
P_Obs2(1,:) = [10 4 rand(1)]; % Initial position
rng('shuffle');
%ang =[0; 2*pi*(rand(mxS,1))];
ang(1) = P_Obs2(1,3);
for iter = 2:maxtime
    if rand(1) > direction_coef
        ang(iter) = 2*pi*(rand(1));
    else
        ang(iter) = ang(iter-1);
    end
end
%genika an afhsoume thn taxythta statherh den exoume na allaxoume kati
poX = cumsum([P_Obs2(1,1);cos(ang) * u_ob3]);
poY = cumsum([P_Obs2(1,2);sin(ang) * u_ob3]);
vel(1) =  sqrt((poY(1,1) - poY(2,1))^2 + (poX(1,1) - poX(2,1))^2);
for i = 2:length(time)-1
    P_Obs2(i,3) = ang(i);
    P_Obs2(i,1) = poX(i,1);    % Update position
    P_Obs2(i,2) = poY(i,1); 
    vel(i) =  sqrt((poY(i,1) - poY(i+1,1))^2 + (poX(i,1) - poX(i+1,1))^2) 
end
  P_Obs2(length(time),1) = poX(length(time),1);    % Update position
  P_Obs2(length(time),2) = poY(length(time),1); 
  P_Obs2(length(time),3) = ang(length(time)) % The heading angle of the vehicle
  vel(length(time)) = 0; 
  vel = vel./dt;

%%



% Robot
radius_robot = 0.5;
P_Robot = zeros(length(time),3);
P_Robot(1,:) = [-16 2 0]; % Initial position
for i = 2:length(time)
    P_Obs_all = [P_Obs1(i,:); P_Obs2(i,:)];
    u_Obs_all = [velocity1; velocity2];
    P_Robot(i,:) = H4_PotentialNavigation(GoalPoint,P_Robot(i-1,:),radius_robot,P_Obs_all,radius_Obs_all,u_Obs_all,gama,ro);
end
P_Robot;
%Plot Trajectories
figure
for t = 2:length(time)
    %plot(P_Obs1(:,1),P_Obs1(:,2),'--b',P_Obs1(t,1),P_Obs1(t,2),'bo',P_Obs2(:,1),  P_Obs2(:,2),'--m',P_Obs2(t,1),P_Obs2(t,2), 'ms');
    plot(P_Robot(:,1),P_Robot(:,2),'r',P_Robot(t,1),P_Robot(t,2),'r*',P_Obs1(:,1),P_Obs1(:,2),'b',P_Obs2(:,1),P_Obs2(:,2),'m',P_Obs2(t,1),P_Obs2(t,2),'ms',P_Obs1(t,1),P_Obs1(t,2),'bo')
    robot_range = animatedline('LineWidth',1,'MarkerSize',ro*10,'Color','r');
    addpoints(robot_range,P_Robot(t,1),P_Robot(t,2))
    robot_range.Marker = 'o';    
    goal = animatedline('Markerface','r','Color','r');
    addpoints(goal,GoalPoint(1),GoalPoint(2))
    goal.Marker = 'o';
    title('Plot Trajectories');
    xlabel('x');
    ylabel('y')
    axis([-17 26 -17 26]);
    %axis([7 13 -7 -1]);
    grid
    axis square
    drawnow
    pause(0.2); %wait 0.01 seconds so the plot is displayed
end
