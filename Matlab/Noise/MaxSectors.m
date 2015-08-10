%# random 4 d array with different size in each dim
A = rand([3,3,3,5]);

%# finds the max of A and its position, when A is viewed as a 1D array
[max_val, position] = max(A(:)); 

%#transform the index in the 1D view to 4 indices, given the size of A
[i,j,k,l] = ind2sub(size(A),position);
indices = cell(1,length(size(A)));

[indices{:}] = ind2sub(size(A),position)