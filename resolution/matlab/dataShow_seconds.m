function [m_] = dataShow_seconds(data, n, text, final_time, delta_t)
% Plots the median of optimality and obtained cost over the time
% n --> number of UAVs to be considered
% text --> a cell with the text describing each method
% Remember:
% data{method}{uavs}{cost,time,plan}{tests}
close all;

if (nargin < 5) 
  delta_t = 0.1;
end


% Time against iterations [parametrized with n_uavs]
color=['b' 'm' 'g' 'k' 'y' 'r' 'b--' 'm--' 'g--' 'k--' 'y--' 'r--'];

% The output will be:
% cost{method}{experiment}{t}

t_vector = [delta_t:delta_t:final_time];

% First get the maximum with each method
maxs = -(ones(length(data{1}{n}{2}), 1)* inf);
mins = ones(length(data{1}{n}{2}), 1)*inf;
for curr_method=1:length(data)
  for m=1:length(data{curr_method}{n}{2})
    for it=1:length(data{curr_method}{n}{2}{m})
      aux{curr_method}(m,it)=data{curr_method}{n}{1}{m}(it);
    end
    maxs(m) = max([maxs(m) max(aux{curr_method}(m, :))]);
    mins(m) = min([mins(m) min(aux{curr_method}(m, :))]);
  end
end
diffs = maxs - mins ;
a = 1./diffs;
b = -mins./diffs;

% get the cost over the time (starting from the maximum cost in each case)
figure;
max_cost = max(maxs);
aux2getMedian = cell(length(data), 1);
for curr_method=1:length(data)
  for m=1:length(data{curr_method}{n}{2}) % For each experiment
    cont_t = 1;
    last_cost = max_cost;
    curr_time = data{curr_method}{n}{2}{m}(1);
    last_time = 0;
    cont_it = 1;
    for t = delta_t:delta_t:final_time
      % First get the first value above
      while (cont_it <= length(data{curr_method}{n}{2}{m}) && data{curr_method}{n}{2}{m}(cont_it) < t)
        last_cost = data{curr_method}{n}{1}{m}(cont_it);
        last_time = data{curr_method}{n}{2}{m}(cont_it);
        cont_it = cont_it + 1;
      end
      % Get the time and cost of the current data used
      if (cont_it <= length(data{curr_method}{n}{2}{m}))
	curr_time = data{curr_method}{n}{2}{m}(cont_it);
	curr_cost = data{curr_method}{n}{1}{m}(cont_it);
	% Interpolation
	cost{curr_method}(m,cont_t) = last_cost + (curr_cost - last_cost)/(curr_time - last_time) * (t - last_time);
	cont_t = cont_t + 1;
	
      end
    end
  end
  median_cost{curr_method}=median(cost{curr_method});
 
  
  plot(t_vector, median_cost{curr_method}, color(curr_method), 'lineWidth', 3);
  hold on;
end

setLabelStyle('Execution time (s)', 'Median of the cost');
legend_handle=legend(text);
set(legend_handle,'fontsize',28, 'location', 'northeast');
axis([0 final_time 0 max_cost]);
hold off;

 
%  % Normalized cost through successive iterations [parametrized with n_uavs]
figure;
for curr_method=1:length(data)
  for m=1:length(data{curr_method}{n}{1})
    for cont_t=1:length(t_vector)
      if isnan(a(m)) == 0
	norm_cost{curr_method}(m,cont_t)=cost{curr_method}(m,cont_t)*a(m) + b(m);
	if isnan(norm_cost{curr_method}(m, cont_t)) == 1
	  norm_cost{curr_method}(m, cont_t) = 0;
	end
      else 
	norm_cost{curr_method}(m, cont_t) = 0;
      end
    end
  end
  median_norm_cost{curr_method}=median(norm_cost{curr_method});
  plot(t_vector, median_norm_cost{curr_method}, color(curr_method), 'lineWidth', 3);
  
  hold all;
end %for curr_method
%    
%  
line([0 final_time], [0.1 0.1], 'Color', 'k', 'LineWidth', 2, 'LineStyle', '--');
axis([0 final_time 0 1]);
setLabelStyle('Execution time (s)', 'Median of the normalized cost');
legend_handle=legend(text);
set(legend_handle,'fontsize',18);
hold off;


%  % Anytime figures will use the data in figures 1 and 2
figure;
for curr_method=1:length(data)
  for i=1:length(t_vector)
    fig4_data{curr_method}(i)=(1-median_norm_cost{curr_method}(i))*100; % Conversion from C* to optimality
  end
  plot(t_vector, fig4_data{curr_method}, color(curr_method), 'lineWidth', 3);
  hold all;
end

setLabelStyle('Execution time (s)', 'Optimality (%)');
axis([0 final_time 0 100]);
legend_handle=legend(text);
set(legend_handle,'fontsize', 18);
hold off;

end
