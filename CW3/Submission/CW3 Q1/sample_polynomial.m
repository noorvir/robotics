%%
% Sample N points at intervals of 3/N on a fitted polynomial
function [value, point] = sample_polynomial(value, pol_coefs, p, t0,...
                                            t_step, tf, point)

    % Prevent sampling the same time twice
    if p > 1
        t0 = t0 + t_step;
    end

    for t = t0 : t_step : tf

        value(point,:) = [pol_coefs(:,1,p) + pol_coefs(:,2,p)*t ...
                                + pol_coefs(:,3,p)*t^2 + pol_coefs(:,4,p)*t^3 ...
                                + pol_coefs(:,5,p)*t^4 + pol_coefs(:,6,p)*t^5]';
        point = point+1;
    end 
    
end