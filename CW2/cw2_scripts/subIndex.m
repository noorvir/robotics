%%
% Helper function to index into a matrix returned by a function call
% Returns sub-matrix subMat at indicies i,j of matrix mat
function subMat = subIndex(mat,i,j) 
    subMat = mat(i,j);
end