function median_time=getMedianTime(data, uav_min, uav_max)
% This function takes the data of a random battery of experiments
% and extracts the median time of each iteration in each case
% data{uavs}{tests}{cost,time,plan}

median_time=cell(uav_max);

% Calculate median time  [parametrized with n_uavs]
for n=uav_min:uav_max
    for m=1:length(data{n})
        for it=1:length(data{n}{m}{2})
            aux_median(m, it)=data{n}{m}{2}(it);
        end
    end
    median_time{n} = median(aux_median);
end

end