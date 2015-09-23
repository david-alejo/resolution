function first_vector=getFirstIteration(data, uav_min, uav_max)
% Returns a vector with the number of the iterations 
% in wich the algorithm has obtained solution at least 90% of times
% data{uavs}{tests}{cost,time,plan}

first_vector = zeros(1,uav_max);

% Aux variable that will contain the first iteration where the algorithm
% found solution in each particular solved problem
curr_first_vec = zeros(1, length(data{2}));



% Number of iterations in each case
n_iterations = length(data{2}{1}{1});


for n=uav_min:uav_max % For each number of UAVs in the system
    for m=1:length(data{n}) % For each particular problem
        min_cost = data{n}{m}{1}(n_iterations); % Store the minimum cost in this problem
        for it=1:n_iterations % For each iteration
            current_cost = data{n}{m}{1}(it); % Current cost
           
            if (current_cost - min_cost < 200)
                curr_first_vec(m) = it; % If difference of the min with the curr cost is not much-->no collision
                break;
            end
           
        end
        
    end

    % We have calculated the iteration number in wich all particular cases
    % have found solution. Let's see in wich number the 90% has found
    % solution
    
    
    for l=1:n_iterations % % in each iteration
        count = 0;
        for m=1:length(data{n}) % For each problem
            if (curr_first_vec(m) <= l) 
                count=count+1; % if the first solution has been obtained count
            end
        end
        if (count >= 0.9*length(data{n}))
            first_vector(n) = l; % If exceeds 90% --> store
            break;
        end
    end
    
end


end