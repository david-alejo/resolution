function mat_trans = translate_matrix(mat, vec)
mat_trans = mat;
for i = 1:length(mat)
    mat_trans(i, :) = mat(i, :) + vec;
end
