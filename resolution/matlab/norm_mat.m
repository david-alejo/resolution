function [ret] = norm_mat(mat)

ret = zeros(length(mat), 1)

for i=1:length(mat)
    ret(i, 1) = norm(mat(i,:));
end