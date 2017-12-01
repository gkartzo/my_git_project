function [x1] = H4_PotentialNavigation(Goal,P_Robot,radius_robot,P_Obs_all,radius_Obs_all,u_Obs_all,gama,ro)

time = [0 1];
[t x] = ode23(@vehicle,time,P_Robot);
x1 = x(end,:);

  function dx = vehicle(t,x)     
      v  = 1; % Robot constant longitudinal velocity
      L = 2; % The wheel base
      
      rG = sqrt((Goal(1) - x(1))^2 + (Goal(2) - x(2))^2);
      if rG < 0.05 % Stop the vehicle when arrives to the goal
          v = 0;
      end
      
      alpha = MultipleObstacles(P_Obs_all,radius_Obs_all,u_Obs_all,Goal,radius_robot,x,v,gama,ro);
      
      K = 2;
      ph = K*(alpha - x(3)); % Orientation Control  ?? 57.2957*
      
      %dx = [v*cosd(ph)*cos(x(3));v*cosd(ph)*sin(x(3));v*sind(ph)/L]; % cosd / cos ??
      dx = [v*cosd(ph); v*sind(ph); v/L];
  end
end
