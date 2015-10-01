function [m_] = box_plot_time_iteration(data, uav_vec, text, last_iteration)
% Makes a box plot of the time to perform one iteration for each method
% n --> number of UAVs to be considered
% text --> a cell with the text describing each method
% Last_iteration: the last iteration to be plotted (optional)
% Remember:
% data{method}{uavs}{cost,time,plan}{tests}

figure;
n_tests = length(data{1}{uav_vec(1)}{1});
time_data = cell(length(data), 1);
hold on;

for it_uav = 1:length(uav_vec)
  n = uav_vec(it_uav);
  for curr_method=1:length(data)
    time_vec = zeros(1, 1);
    last_iteration = length(data{curr_method}{n}{2}{1});
    cont = 1;
    for m=1:length(data{curr_method}{n}{2})
      time_vec(cont) = data{curr_method}{n}{2}{m}(1);
      cont = cont + 1;
      for it=2:last_iteration
	time_vec(cont) = data{curr_method}{n}{2}{m}(it) - data{curr_method}{n}{2}{m}(it - 1);
	cont = cont + 1;
      end
    end
    time_data{curr_method} = time_vec;
  end
  
  cellbplot(time_data);
end
