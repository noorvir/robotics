function [translation_error, rotation_error] = calculate_pose_error(estimated_traj, desired_traj)

    [M, N] = size(desired_traj);
    %For rotation error, you have to calculate relative error between two
    %rotation matrices and use rodrigues function to compute the error
    %in term of 3x1 vector. 
    %The template of rodrigues function is given below
    %R = rodrigues(r) and r = rodrigues(R), where
    %r: 3x1 vector (axis of rotation with the norm equal to angle rotation)
    %R: 3x3 rotation matrix
    
    translation_error  = zeros(1,M);
    rotation_error = zeros(1,M);
    
    for i = 1: M
        if (N == 2)
            %Implement code for calculating pose error in 4R-planar robot here

            transVec = estimated_traj(1:2,4,i);
            translation_error(1,i) = norm(desired_traj(i,1:2)' - transVec);
            rotation_error = zeros(1, M);
        else
            %Implement code for calculating pose error in youbot and KUKA here
            rotVec = rodrigues(estimated_traj(1:3,1:3,i));
            transVec = estimated_traj(1:3,4,i);
            
            % Error in translation and rotation calculated as a norm
            translation_error (1,i) =  norm(desired_traj(i,1:3)' - transVec); 
            rotation_error (1,i) = norm(desired_traj(i,4:6)' - rotVec);%randn(1, M);
        end
    end

end