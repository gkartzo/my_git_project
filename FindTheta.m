function [theta] = FindTheta(P_Robot,P_Obs)
theta = atand((P_Obs(2) - P_Robot(2))/(P_Obs(1) - P_Robot(1)));
end

