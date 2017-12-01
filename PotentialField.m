function [phi] = PotentialField(x,Goal,Obs,r_security)
      KG = 1;
      
      % Goal - Obstacle
      rG_Obs = sqrt((Obs(1)-Goal(1))^2 + (Obs(2)-Goal(2))^2);
      b = r_security / (r_security + rG_Obs);
      
      % Goal Point - Robot
      rG = sqrt((Goal(1) - x(1))^2 + (Goal(2) - x(2))^2);
      EGx = KG*(Goal(1) - x(1))/rG^2;
      EGy = KG*(Goal(2) - x(2))/rG^2;
      
      % Obstacle - Robot
      rObs = sqrt((Obs(1) - x(1))^2 + (Obs(2) - x(2))^2);
      EObsx = -b*(Obs(1) - x(1))/rObs^2;
      EObsy = -b*(Obs(2) - x(2))/rObs^2;
      
      Ex = (EGx + EObsx);
      Ey = (EGy + EObsy);
      
      phi = atand(Ey/Ex);
end