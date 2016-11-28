clear; close all; clc;

n = 10 % Matrix dimensions to try
timeCF = zeros(1,n);
timedet = zeros(1,n);
fprintf('Calculating determinant without using LU-decomposition');

for i = 1:n
    mat = randi(i,i);
    tic
    d = det_cofactorExp(mat);
    timeCF(i) = toc;
    tic
    d_det = det(mat);
    timedet(i) = toc;
    fprintf('Determinant of %d x %d matrix calculated using det_cofactorExp:%f\n', i,i, d);
    fprintf('Determinant of %d x %d matrix calculated using MATLAB det command:%f\n', i,i, d_det);
end    

figure
plot(1:n, log10(timeCF), 1:n, log10(timedet))
title('Logorithmic Plot of Time Taken to Calculate Determinant')
xlabel('Dimension of Square Matrix') % x-axis label
ylabel('Log time (s)') % y-axis label

function d = det_cofactorExp(mat)
    % Calcuate determinant manually using recursion
    d = 0;
    for i = 1 : size(mat, 1)
        % Expand along the first column
        j = 1;
        % Delete the ith row and jth column to get submatrix
        sub_mat = mat;
        sub_mat(i,:) = [];
        sub_mat(:, j) = [];
        
        if size(mat, 1) > 1
            % Compute determinant by recursive formula
            d = d + (-1)^(i+j) * mat(i, j) * det_cofactorExp(sub_mat);
        else
            d = mat(i,j);
        end  
    end
end


function det_lu(mat)
    % Calculate the determinant using ____ algorithm
end
