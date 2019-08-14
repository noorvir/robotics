%%
% Fit either a regular quintic polynomial or a cubic spline through a set
% of points with 4th order polynomials at the end in order to constrain the
% acceleration and velocity
%
% @mode:        "quintic" or "spline"
% @points:      points to fit though - must be size 2 for quintic
% @cnst:        struct containing velocity and acceleration constraints
% @t0:          starting time for point
% @tf:          end time for point
function [coefs] = fit_polynomial(mode, points, cnst, t0, tf)

    if strcmp(mode, 'quintic')
        % Fit quintic
        
        if size(points, 2) ~= 2
            error('Number of points should be equal to 2 for quintic fit');
        end
        
        % Number of dimensions of data
        dim = size(points,1);
        
        % Coefficient matrix - 6 coefficients for each dimension (x,y,z)
        coefs = zeros(dim, 6);
        
        % 1st Derivate (Velocity) constraints
        v_i = cnst.vel(1);
        v_f = cnst.vel(2);
        
        % 2nd Derivate (Acceleration) constraints
        a_i = cnst.acc(1);
        a_f = cnst.acc(2);
        
        % Compute the interpolating polynomial for each dimension
        % independently
        for dim = 1:size(points,1)
             % Coefficient matrix
            A = [1, t0, t0^2, t0^3, t0^4, t0^5;...
                 0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;...
                 0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;...
                 1, tf, tf^2, tf^3, tf^4, tf^5;...
                 0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;...
                 0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

            b = [points(dim,1);v_i;a_i;points(dim,2);v_f;a_f];

            % Solve for the coefficients of the quintic polynomial
            coefs(dim,:) = linsolve(A, b)';
        end
        
    elseif strcmp(mode, 'spline')
        % Fit spline
        
        
        
    else
        error('Unknown fit method specifies');
    end
end