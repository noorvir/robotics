function [t1, REV] = stage_angle(angle)

    REV = false;
    
    if angle > 0
        t1 = angle - pi;
        
        if angle < deg2rad(9)
            t1 = angle;
            REV = true;
        end
    else
        t1 = pi+angle;
        
        if  angle > - deg2rad(9)
            t1 = angle;
            REV = true;
        end
    end
        