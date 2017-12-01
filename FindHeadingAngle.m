function [theta] = FindHeadingAngle(P_Obs1,center)
if P_Obs1(1) < 0
    theta = atand((P_Obs1(2) - center(2)) / (P_Obs1(1) - center(1))) + 90;
else
    theta = atand((P_Obs1(2) - center(2)) / (P_Obs1(1) - center(1))) - 90;
end

end

