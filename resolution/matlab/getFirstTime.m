function first_vector=getFirstIteration(data, uav_min, uav_max)
% Returns a vector with the number of the iterations 
% in wich the algorithm has obtained solution at least 90% of times
% data{uavs}{tests}{cost,time,plan}

% Median of the minimum cost against iterations [parametrized with n_uavs]
figure;
curr_first_vec = zeros(1,m);
for n=uav_min:uav_max
    for m=1:length(data{n})
        min_cost = data{n}{m}{1}(length(data{n}{m}{1}));
        for it=1:length(data{n}{m}{1})
            current_cost = data{n}{m}{1}(it);
           
            if (current_cost - min_cost < 125 )
                curr_first_vec(m) = it;
                break;
            end
           
        end
        
    end

    first_vector(n) = mean(curr_first_vec);
end


end