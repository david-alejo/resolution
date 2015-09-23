function plot_distance(pos_1, pos_2, dim, color)

n=min(length(pos_1), length(pos_2));

repr = zeros(n, 1);
repr_2 = zeros(n, 1);
for i=1:n
    if (dim == 3)
        repr(i) = sqrt( power(pos_1(i, 2) - pos_2(i, 2), 2) + power(pos_1(i, 3) - pos_2(i, 3), 2) + power(pos_1(i, 4) - pos_2(i, 4), 2));
        
    elseif (dim == 2)
        repr(i) = sqrt( power(pos_1(i, 2) - pos_2(i, 2), 2) + power(pos_1(i, 3) - pos_2(i, 3), 2));
        
    else
        repr(i) = sqrt( power(pos_1(i, 2) - pos_2(i, 2), 2) + power(pos_1(i, 3) - pos_2(i, 3), 2));
        repr_2(i) = abs(pos_1(i, 4) - pos_2(i, 4));
    end
end



if (n == length(pos_1))
    plot (pos_1(:,1), repr, color);
else 
    plot (pos_2(:,1), repr, color);
end
setLabelStyle('t(s)', 'separation(m)');
switch (dim) 
    case 3
        legend_handle = legend('3D separation');
    case 2
        legend_handle = legend('2D separation');
    otherwise
        hold on;
        if (n == length(pos_1))
	  plot (pos_1(:,1), repr_2, color);
	else 
	  plot (pos_2(:,1), repr_2, color);
	end
        
        legend_handle = legend('Horizontal separation', 'Vertical separation');
end
set(legend_handle,'fontsize',14);

end