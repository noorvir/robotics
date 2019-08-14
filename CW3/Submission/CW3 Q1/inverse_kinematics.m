%%
% Set angles values from closed form solution. Wrap to [-pi, pi] if 
% needed. 
function theta = inverse_kinematics(tHom, robot)
    
    % Initialise robot properties
    limits = [deg2rad(-169) deg2rad(169);
              deg2rad(-65) deg2rad(90);
              deg2rad(-151) deg2rad(146);
              deg2rad(-102.5) deg2rad(102.5);
              deg2rad(-167.5) deg2rad(167.5);]';
    
    % Robot parameters (shorthand variables)
    d1 = robot.d(1);
    d5 = robot.d(5);
    a1 = robot.a(1);
    a2 = robot.a(2);
    a3 = robot.a(3);
    
    % Initialise variables
    theta = zeros(4, 5);
    R1 = zeros(2,1);
    R2 = zeros(2,1);
    temp = zeros(2,1);
    
    %=================================================
    % THETA 1
    %=================================================
    %  There are two values for theta 1 (± pi)
    t1(1) = atan2(tHom(2,4),tHom(1,4));   
    t1(2) = t1(1) - sign(t1(1)) * pi;
    
    count = 1;
    
    for i = 1:2
        
        %=================================================
        % THETA 5
        %=================================================
        t5 = atan2(tHom(1,1)* sin(t1(i)) - tHom(2,1)*cos(t1(i)),...
                    tHom(1,2) * sin(t1(i)) - tHom(2,2)*cos(t1(i)));
                
        t234 = atan2(cos(t5) * tHom(3,1) - sin(t5) * tHom(3,2), -tHom(3,3));
        
%         t234 = atan2(-tHom(3,2)/sin(t5),-tHom(3,3));
        
        % Temporary variables
        R1(i) = tHom(2,4)/sin(t1(i)) - d5 * sin(t234) - a1;
        R2(i) = tHom(3,4) + d5 * cos(t234) - d1;
        temp(i) = (R1(i)^2 + R2(i)^2 - (a2^2 + a3^2))/(2*a2*a3);
        
        for j = 1:2

            if temp(i) > 1
                break;
            end
            
            %=================================================
            % THETA 3
            %=================================================
            t3 = (-1)^(j)*acos(temp(i));

            % More Temporary variables
            A = a3 * cos(t3) + a2;
            B = a3 * sin(t3);

            %=================================================
            % THETA 2
            %=================================================
            t2 = atan2(R2(i),R1(i)) - atan2(B,A);

            %=================================================
            % THETA 4
            %=================================================
            t4 = t234 - t2 - t3;

            % Account for the offset in t2 and t4 - this was ignored earlier in
            % order to simplify notation/calculation
            t4 = t4 - pi/2;
            t2 = t2 - pi/2;
            
            theta(count,:) = [t1(i), t2, t3, t4, t5];
            
            count = count +1;
        end
    end
    
    % Wrap to [-pi,pi]
    theta = wrapToPi(theta);
    
    theta = check_joint(theta, limits);
end
        

function t = check_joint(theta, limits)

    t = [];
    for i = 1:4
        if all(theta(i,:) >= limits(1,:) & theta(i,:) <= limits(2,:))
            t = [t; theta(i,:)];
        else
            t = [t; [nan, nan, nan, nan, nan]];
        end
    end
end
