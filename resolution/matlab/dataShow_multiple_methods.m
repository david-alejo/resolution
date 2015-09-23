function [m_] = dataShow_multiple_methods(data, n, text, last_iteration)
% n --> number of UAVs to be considered
% text --> a cell with the text describing each method
% Last_iteration: the last iteration to be plotted (optional)
% Remember:
% data{method}{uavs}{cost,time,plan}{tests}
close all;
%  if (nargin < 4) 
%    last_iteration = length(data{curr_method}{n}{2}{1});
%  end

%  % Time against n_uavs (boxplot)
%  figure;
%  for n=uav_min:uav_max
%      for m=1:length(data{curr_method}{n}{2})
%        if (length(data{curr_method}{n}{2}{m})>0)
%          fig1_data(m,n)=data{curr_method}{n}{2}{m}(length(data{curr_method}{n}{2}{m}));
%        end
%      end
%  end
%  boxplot(fig1_data,'symbol','');
%  setLabelStyle('Number of UAVs', 'Execution time after 100 iterations (s)');

% Time against iterations [parametrized with n_uavs]
color=['b' 'm' 'g' 'k' 'y' 'r'];

figure;
aux2getMedian = cell(length(data), 1);
for curr_method=1:length(data)
  last_iteration = length(data{curr_method}{n}{2}{1});
  aux2getMedian=zeros(length(data{curr_method}{n}{2}), last_iteration);
  for m=1:length(data{curr_method}{n}{2})
    for it=1:last_iteration
      aux2getMedian(m,it)=data{curr_method}{n}{2}{m}(it);
    end
  end
  fig1_data{curr_method}{2}=median(aux2getMedian);
  fig1_data{curr_method}{1}=1:length(fig1_data{curr_method}{2});
  plot(fig1_data{curr_method}{1}, fig1_data{curr_method}{2}, color(curr_method), 'lineWidth', 3);
  hold on;
end

setLabelStyle('Number of iterations', 'Median of the execution time(s)');
legend_handle=legend(text);
set(legend_handle,'fontsize',18, 'location', 'northwest');
hold off;


% Median of the minimum cost against iterations [parametrized with n_uavs]
figure;
aux2getNormMedianCost = cell(length(text), 1);
for curr_method=1:length(data)
    aux2getNormMedianCost{curr_method}=zeros(length(data{curr_method}{n}{1}),last_iteration);
    for m=1:length(data{curr_method}{n}{1})
        for it=1:last_iteration
            aux2getNormMedianCost{curr_method}(m,it)=data{curr_method}{n}{1}{m}(it);
        end
        
    end
    fig2_data{curr_method}{2}=median(aux2getNormMedianCost{curr_method});
    fig2_data{curr_method}{1}=1:length(fig2_data{curr_method}{2});
    plot(fig2_data{curr_method}{1}, fig2_data{curr_method}{2}, color(curr_method), 'lineWidth', 3);
    hold on;
end
setLabelStyle('Number of iterations', 'Median of the minimum cost (m)');
[legend_handle]=legend(text);
set(legend_handle,'fontsize',18);
hold off;

% Normalized cost through successive iterations [parametrized with n_uavs]
figure;

% First get the maximum with each method
maxs = -(ones(length(data{curr_method}{n}{2}), 1)* inf);
mins = ones(length(data{curr_method}{n}{2}), 1)*inf;
for curr_method=1:length(data)
  for m=1:length(data{curr_method}{n}{2})
    for it=1:last_iteration
      aux2getNormMedianCost{curr_method}(m,it)=data{curr_method}{n}{1}{m}(it);
    end
    maxs(m) = max([maxs(m) max(aux2getNormMedianCost{curr_method}(m, :))]);
    mins(m) = min([mins(m) min(aux2getNormMedianCost{curr_method}(m, :))]);
  end
end
diffs = maxs - mins ;
a = 1./diffs;
b = -mins./diffs;


for curr_method=1:length(data)
  for m=1:length(data{curr_method}{n}{1})
    for it=1:last_iteration
      if isnan(a(m)) == 0
	aux2getNormMedianCost{curr_method}(m,it)=aux2getNormMedianCost{curr_method}(m,it)*a(m) + b(m);
	if isnan(aux2getNormMedianCost{curr_method}(m,it)) == 1
	  aux2getNormMedianCost{curr_method}(m,it) = 0;
	end
      else 
	aux2getNormMedianCost{curr_method}(m,it) = 0;
      end
    end
  end
  fig3_data{curr_method}{2}=median(aux2getNormMedianCost{curr_method});
  fig3_data{curr_method}{1}=1:length(fig3_data{curr_method}{2});
  plot(fig3_data{curr_method}{1}, fig3_data{curr_method}{2}, color(curr_method), 'lineWidth', 3);
  
  hold all;
end %for curr_method
  

line([1 it], [0.1 0.1], 'Color', 'k', 'LineWidth', 2, 'LineStyle', '--');
setLabelStyle('Number of iterations', 'Median of the normalized cost');
legend_handle=legend(text);
set(legend_handle,'fontsize',18);
hold off;


% Anytime figures will use the data in figures 1 and 2
figure;
for curr_method=1:length(data)
  fig4_data{curr_method}{2}(1) = 0;
  fig4_data{curr_method}{1}(1) = 0;
  for i=1:last_iteration
    fig4_data{curr_method}{2}(i+1)=(1-fig3_data{curr_method}{2}(i))*100; % Conversion from C* to optimality
    fig4_data{curr_method}{1}(i+1)=fig1_data{curr_method}{2}(i); % The time in seconds
  end
  plot(fig4_data{curr_method}{1}, fig4_data{curr_method}{2}, color(curr_method), 'lineWidth', 3);
  hold all;
end

setLabelStyle('Execution time (s)', 'Optimality (%)');
legend_handle=legend(text);
set(legend_handle,'fontsize', 18);
hold off;

end
