function [m_] = dataShow(data, uav_min, uav_max, text, last_iteration, iteration)
% Last_iteration: the last iteration to be plotted
% Iteration: Refers to the iteration in which the mean and std of the ex_time and cost will be calculated
%              (it is a vector for a diff iteration might be needed for each UAV number)
% Remember:
% data{uavs}{cost,time,plan}{tests}
close all;
if (nargin < 5) 
  last_iteration = length(data{uav_min}{2}{1});
end
if (nargin < 4) 
  text = ' UAVs';
end

text_uav = getUAVText(uav_min, uav_max, text);

%  % Time against n_uavs (boxplot)
%  figure;
%  for n=uav_min:uav_max
%      for m=1:length(data{n}{2})
%        if (length(data{n}{2}{m})>0)
%          fig1_data(m,n)=data{n}{2}{m}(length(data{n}{2}{m}));
%        end
%      end
%  end
%  boxplot(fig1_data,'symbol','');
%  setLabelStyle('Number of UAVs', 'Execution time after 100 iterations (s)');

m_=cell(2,1);
m_{1}=zeros(uav_max,2);
m_{2}=zeros(uav_max,2);

if (nargin < 5) 
  iteration = zeros(uav_max);
end

% Time against iterations [parametrized with n_uavs]
color=['b' 'm' 'g' 'k' 'y' 'b--' 'm--' 'g--'];

figure;
for n=uav_min:uav_max
    aux2getMedian=zeros(length(data{n}{2}), last_iteration);
    for m=1:length(data{n}{2})
        for it=1:last_iteration
            aux2getMedian(m,it)=data{n}{2}{m}(it);
        end
    end
    fig1_data{n}{2}=median(aux2getMedian);
    fig1_data{n}{1}=1:length(fig1_data{n}{2});
    plot(fig1_data{n}{1}, fig1_data{n}{2}, color(n), 'lineWidth', 3);
    hold on;
    % Get the mean time in the last iteration
    if (nargin < 6) 
      iteration(n) = length(data{n}{1}{1});
    end

    m_{1}(n,1)=mean(aux2getMedian(:, iteration(n)));
    m_{1}(n,2)=std(aux2getMedian(:, iteration(n)));
end

setLabelStyle('Number of iterations', 'Median of the execution time(s)');
legend_handle=legend(text_uav);
set(legend_handle,'fontsize',18, 'location', 'northwest');
hold off;


% Median of the minimum cost against iterations [parametrized with n_uavs]
figure;
for n=uav_min:uav_max
    aux2getNormMedianCost=zeros(length(data{n}{1}),last_iteration);
    for m=1:length(data{n}{1})
        for it=1:last_iteration
            aux2getNormMedianCost(m,it)=data{n}{1}{m}(it);
        end
        
    end
    fig2_data{n}{2}=median(aux2getNormMedianCost);
    fig2_data{n}{1}=1:length(fig2_data{n}{2});
    plot(fig2_data{n}{1}, fig2_data{n}{2}, color(n), 'lineWidth', 3);
    hold on;
    
    m_{2}(n,1)=mean(aux2getNormMedianCost(:, iteration(n)));
    m_{2}(n,2)=std(aux2getNormMedianCost(:, iteration(n)));
end
setLabelStyle('Number of iterations', 'Median of the minimum cost (m)');
[legend_handle]=legend(text_uav);
set(legend_handle,'fontsize',18);
hold off;

% Normalized cost through successive iterations [parametrized with n_uavs]
figure;
for n=uav_min:uav_max
    for m=1:length(data{n}{1})
        for it=1:last_iteration
            aux2getNormMedianCost(m,it)=data{n}{1}{m}(it);
        end
        
    end
    maxs = max(aux2getNormMedianCost'); %Maximum of all iters for each test
    mins = min(aux2getNormMedianCost'); 
    diffs = maxs - mins ;
    a = 1./diffs;
    b = -mins./diffs;
    for m=1:length(data{n}{1})
        for it=1:last_iteration
            if isnan(a(m)) == 0
	      aux2getNormMedianCost(m,it)=aux2getNormMedianCost(m,it)*a(m) + b(m);
	      if isnan(aux2getNormMedianCost(m,it)) == 1
	        aux2getNormMedianCost(m,it) = 0;
	      end
	    else 
	      aux2getNormMedianCost(m,it)=0;
            end
        end
    end

    
    fig3_data{n}{2}=median(aux2getNormMedianCost);
    fig3_data{n}{1}=1:length(fig3_data{n}{2});
    plot(fig3_data{n}{1}, fig3_data{n}{2}, color(n), 'lineWidth', 3);
    
    hold all;
    
end
line([1 it], [0.1 0.1], 'Color', 'k', 'LineWidth', 2);
setLabelStyle('Number of iterations', 'Median of the normalized cost');
legend_handle=legend(text_uav);
set(legend_handle,'fontsize',18);
hold off;


% Anytime figures will use the data in figures 1 and 2
figure;
for n=uav_min:uav_max
  fig4_data{n}{2}(1) = 0;
  fig4_data{n}{1}(1) = 0;
  for i=1:last_iteration
    fig4_data{n}{2}(i+1)=(1-fig3_data{n}{2}(i))*100; % Conversion from C* to optimality
    fig4_data{n}{1}(i+1)=fig1_data{n}{2}(i); % The time in seconds
  end
  plot(fig4_data{n}{1}, fig4_data{n}{2}, color(n), 'lineWidth', 3);
  hold all;
end
setLabelStyle('Execution time (s)', 'Optimality (%)');
legend_handle=legend(text_uav);
set(legend_handle,'fontsize',18);
hold off;
