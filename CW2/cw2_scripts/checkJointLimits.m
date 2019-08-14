% Checks is the input angle lies within the permissable range for the
% manipulator angles. Returns two variables:
% nTheta    -   new wrapped theta
% fTheta    -   if the wrapped theta is still outside permissable range
%               keep using original (wrapped theta for further calculations
%               but set the final value to th max joint limit
%               eg. if permissible joint value is in the range -150 < t < 150
%               and t from closed form solution is 170 then set nTheta =
%               170 but fTheta = 150.
function [nTheta, fTheta] = checkJointLimits(theta, jt, limits)
    
              
    nTheta = theta;
    fTheta = theta;
    
    if theta < limits(jt,1) || theta > limits(jt,2)

        % wrap theta to [-pi,pi]
        nTheta = wrapToPi(theta);

        % check if theta is still outside permissible bounds
        if nTheta > limits(jt,2)
            fTheta = limits(jt,2); 

        elseif nTheta < limits(jt,1)
            fTheta = limits(jt,1);
        end

    end 
end