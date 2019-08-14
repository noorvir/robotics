%% 
% Calculate the rotation interpolation between two poses and return it as a
% rotation matrix R
function [rotation_profile] = slerp(rotation_profile, slerp_t, theta, q1, q2)

    % Robotics Toolbox doesn't handle row vector of Quaternions for some
    % reason - tranform to column vector
    slerp_t = slerp_t';
    
    % Interpolate
    term1 = sin((1-slerp_t).*theta)./sin(theta);
    term2 = sin(slerp_t.*theta)./sin(theta);
    
    q_s = term1 .* q1.s + term2 .* q2.s;
    q_v = bsxfun(@times,q1.v,term1') + bsxfun(@times,q2.v,term2');
    
    for i = 1:size(q_s,2)
        q(i) = Quaternion([q_s(i), q_v(i,:)]);
        q(i) = q(i)/q(i).norm;
    end
    
    % Convert to rotation matrix
    R = q.R;
    rotation_profile = cat(3, rotation_profile,R);
end