function showPlans(wps, dimen)
% Dimen means the dimension of the obtained waypoints. 2D or 3D plot is used in order to represent it properly.

figure;


color = [ 'r' 'g' 'b' 'k' 'c'];

hold on;
for n=1:length(wps)
  plan=wps{n}(:,:);

      % Calculate the index to the current color
      index = mod(n, length(color)) + 1;
        
      % Represent the flight plan
      current_color=color(index);
      % Regular plot

      if (dimen == 3) 
        plot3(plan(:,1), plan(:,2), plan(:,3), current_color, 'Linewidth',2);
      end
      if (dimen == 2)
              plot(plan(:,1), plan(:,2), current_color, 'Linewidth',2);
      end
      
      
      % Represent the goal waypoint, marking it with an x
%        color_x = [color(index) 'x'];
%        if (dimen == 3) 
%          plot3 (plan(2,1), plan(2,2), plan(2,3), color_x, 'MarkerSize', 20);
%        end
%        if (dimen == 2)
%                plot (plan(2,1), plan(2,2), color_x, 'MarkerSize', 20);
%        end
end

for n=1:length(wps)
  plan=wps{n};

  % Calculate the index to the current color
  index = mod(n, length(color)) + 1;
        
  % Represent the flight plan
  current_color=color(index);
%    represent_arrow(plan, dimen, current_color, 0.5)
end 
%axis([3 11 3 11 0 3]);
%title('Initial Fligth Plans','fontsize',16,'fontweight','b');
xlabel('X(m)', 'fontsize',16);
ylabel('Y(m)','fontsize', 16);

if (dimen == 3) 
    zlabel('Z(m)','fontsize', 16);
end
set(gca, 'FontSize', 16);


for i=1:length(wps)
    text_uav{i} = ['UAV ' num2str(i)];
end
legend_handle=legend(text_uav);

set(legend_handle,'fontsize',14);

hold off;


end
