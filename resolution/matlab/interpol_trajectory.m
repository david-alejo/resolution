%%% Interpolates a trajectory with a maximum distance between points


function [new_trajectory] = interpol_trajectory(traj, max_distance)

% Copy the first point
new_trajectory = traj(1,:);
cont = 2;
for i=1:length(traj)-1
    %i
    inc = traj(i + 1,:) - traj(i,:);
    l = norm(inc);
    % Check for interpolation
    if (l > max_distance)
        n = ceil(l/max_distance);
        for j=1:(n-1)
            new_trajectory(cont,:) = traj(i,:) + inc * (j/n);
            %new_trajectory(cont,:)
            cont = cont + 1;
        end
    end
    new_trajectory(cont,:) = traj(i + 1, :);
    %new_trajectory(cont,:)
    cont = cont + 1
end

end