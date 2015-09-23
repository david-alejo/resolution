function experimentShow(data, dimension, desc_start, desc_end)

%'data' must be a cell matrix of the logs loaded
%'desc' means de number of values descarted in the end of the experiment of each UAV has to be of the same size of data


color_ = ['g' 'r' 'b' 'k'];
figure;
hold on;
for i=1:length(data)
    i
    index = mod(i,length(color_)) + 1;
    color_dotted = [color_(index) '--'];
    
    if (desc_start(i) + desc_end(i) < length(data{i}(:,1)))
        init_t = desc_start(i);
        max_t = length(data{i}(:,1)) - desc_end(i);
    end
    
    if (dimension == 3)
        plot3(data{i}(init_t:max_t,1), data{i}(init_t:max_t,3), data{i}(init_t:max_t,5), color_dotted);
        plot3(data{i}(init_t:max_t,2), data{i}(init_t:max_t,4), data{i}(init_t:max_t,6), color_(index));
    else 
        plot (data{i}(init_t:max_t,1), data{i}(init_t:max_t,3), color_dotted);
        plot(data{i}(init_t:max_t,2), data{i}(init_t:max_t,4), color_(index));
    end
    
end

hold off;

% Set parameters
xlabel('X(m)', 'fontsize',16);
ylabel('Y(m)','fontsize', 16);
% If necessary, set the zlabel
if (dimension == 3)
  zlabel('Z(m)','fontsize', 16);

%Legend
for i=1:length(data)
    text_uav{i} = ['UAV ' num2str(i)];
end

%Axis font size
ax = gca;
set(gca, 'Fontsize', 16);

end