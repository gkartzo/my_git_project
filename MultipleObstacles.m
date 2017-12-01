function  [fi]= MultipleObstacles(P_Obs_all,radius_Obs_all,u_Obs_all,Goal,radius_robot,P_Robot,v_Robot,gama,ro)
% ro is the maximum range in which robots can scan the workspace
% Goal is the same for all obstacles in one robot.
m = 0;
sum_m = 0;
grad_sum = 0;
fi = 0;

d = zeros(length(radius_Obs_all),1);

for i = 1: length(radius_Obs_all)
    d(i) = sqrt((P_Robot(1)-P_Obs_all(i,1))^2 + (P_Robot(2)-P_Obs_all(i,2))^2);
    theta = FindTheta(P_Robot,P_Obs_all(i,:));
    r_security = checkCollision(P_Robot,v_Robot,radius_robot,P_Obs_all(i,:),u_Obs_all(i),radius_Obs_all(i),gama,theta);
    if d <= ro
        m = 1/(d - r_security);
    else
        m = 0;
    end
    sum_m = m + sum_m;
    e = PotentialField(P_Robot,Goal,P_Obs_all(i,:),r_security);
    grad_sum = (e * m) + grad_sum;
end   

if sum_m ~= 0
    fi = grad_sum/sum_m;
else
    % find the closer obstacle
    min = d(1);
    index_min = 1;
    for i = 2: length(radius_Obs_all)
        if min > d(i)
            min = d(i);
            index_min = i;
        end
    end
    fi = PotentialField(P_Robot,Goal,P_Obs_all(index_min,:),radius_robot + radius_Obs_all(index_min)); 
    %fi = 0;
end

end

