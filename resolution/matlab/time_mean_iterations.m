function [ret]=time_mean_iterations(data, uav_min, uav_max, iterations)
% Remember:
% data{uavs}{cost,time,plan}{tests}
% ret{n,o,1} = mean_time
% ret{n,o,2} = std_time
% ret{n,o,3} = mean_cost
% ret{n,o,4} = std_cost

ret = zeros(uav_max,length(iterations),5);
for n=uav_min:uav_max
    
    for o=1:length(iterations)
      values = zeros(length(data{n}{2}),1);
      costs = zeros(length(data{n}{2}),1);
      for m=1:length(data{n}{2})
        if length(data{n}{2}{m}) >= iterations(o)
	  values(m) = data{n}{2}{m}(iterations(o));
	  costs(m) = data{n}{1}{m}(iterations(o));
	end
      end
      ret(n,o,1) = mean(values); 
      ret(n,o,2) = std(values);
      ret(n,o,3) = mean(costs);
      ret(n,o,4) = std(costs);
      ret(n,o,5) = median(costs);
  end

end