clear all; close all;
% define the System of Linear equations
A_1 = [1 2 -1; -2 1 -3; 0 1 3];
b_1 = [-5; -10; 4];

A_2 = [-5 4 -1; -4 1 -3; 11 1 10];
b_2 = [-30; -13; 21];

% Define variables for experiment
interval = 0.001 : 0.001 : 0.01;

% Number of iterations for a given noise amplitude
n = 100;
% Dimension of random noise vector
noiseD = [3,1];

fprintf('Condition Number for matrix A1: %f\n', cond(A_1));
fprintf('Condition Number for matrix A2: %f\n', cond(A_2));
fprintf('Ground truth value for x1 and x2:\n');

% ground truth value of x
x_gt1 = linsolve(A_1, b_1) %#ok<NOPTS>
x_gt2 = linsolve(A_2, b_2) %#ok<NOPTS>

% Call function to get sensitivity of the matrix as a function of noise
% amplitude
[X1, sens_A1_l1, sens_A1_l2] = sensitivity(A_1, b_1, interval, n, x_gt1, noiseD, 'A_1');
[X2, sens_A2_l1, sens_A2_l2] = sensitivity(A_2, b_2, interval, n, x_gt2, noiseD, 'A_2');

% Plot the results
figure
plot(interval,sens_A1_l2,interval,sens_A2_l2)
title('Magnititude of Error (L2-norm) as a Function of Input Noise')
xlabel('Noise Amplitude') % x-axis label
ylabel('Error - L1-norm') % y-axis label
legend('A_1','A_2')

% Plot the results
figure
plot(interval,sens_A1_l1,interval,sens_A2_l1)
title('Magnititude of Error (L1-norm) as a Function of Input Noise')
xlabel('Noise Amplitude') % x-axis label
ylabel('Error - L1-norm') % y-axis label
legend('A_1','A_2')


function [X, mean_error_l1, mean_error_l2] = sensitivity(A, b, interval, n, x_gt, noiseD, label)
    
    X = zeros(10, 100, 3);
    count = 1;
    mean_error_l1 = zeros(1, 10);    %l1 norm
    mean_error_l2 = zeros(1, 10);    %l2 norm
    
    for noise_amp = interval
        error_l1 = 0;
        error_l2 = 0;
        
        % Average over experiment iterations
        for it = 1 : n
            noise = noise_amp * rand(noiseD);
            x_it = linsolve(A, b + noise);
            
            % Collect data for scatter plot
            X(count, it, 1) = x_it(1); 
            X(count, it, 2) = x_it(2);
            X(count, it, 3) = x_it(3);

            % Add up the error for each experiment to take the mean
            error_l1 = error_l1 + norm(x_it - x_gt,1);
            error_l2 = error_l2 + norm(x_it - x_gt); 
        end
        mean_error_l1(count) = error_l1/n;
        mean_error_l2(count) = error_l2/n;

        count = count + 1;
    end
    % Draw a 3D-scatter plot of the x vector as a function of the amplitude of noise
    % Note that the dimensions of the random noise generation (noiseD) matter.
    % An equal perturbation in all direction might give you the magnitutde measure
    % of sensitivity but not the spread. 
    % Modelling the type of noise might also be useful in different
    % scenarios. For example a gyroscopic drift over time might be equal
    % for all axes but the precision at any given point in time might be
    % different in different directions.
    figure
    % Reshape the data matrix for all experiment points into a vector
    scatter3(reshape(X(:,:,1),1,n*10), reshape(X(:,:,2),1,n*10), reshape(X(:,:,3),1,n*10), 'filled');
    title('x as a Function of Input Noise')
    xlabel('x-axis') % x-axis label
    ylabel('y-axis') % y-axis label
    zlabel('z-axis') % y-axis label
    legend(label)
end


    