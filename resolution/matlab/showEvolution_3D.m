function show_Evolution_3D(wps, dimen)
% Dimen means the dimension of the obtained waypoints. 2D or 3D plot is used in order to represent it properly.

figure;


represent = [ 15 30 45 60];

% This definition of color allows changes in the deep of the color
%color = [ 1 0 0; 0.5 1 0.5; 0.9 0 0.9; 1 1 1];

color = [ 'r' 'g' 'b' 'k' ];

hold on;

%axis([3 11 3 11]);
title('Solution');
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
hold off;

%figure;
hold on;
for n=represent
  for uav=1:length(wps{n}(:,1,1))
      plan(:,:)=wps{n}(uav,:,:);
      
      
        
      % Calculate the index to the current color
      index = mod(uav, length(color)) + 1;
        
      % Represent the flight plan
      
      %Changes the color
      %new_color = 1 - 0.5*color(index,:)*(((length(wps)-n)/length(wps))^2)
      
      % Plot with changing color
      %plot3(plan(:,1), plan(:,2), plan(:,3), 'Color',new_color, 'Linewidth',2);
      
      
      % Calculate the type of line and add to current color
      current_color = color(uav);
      for p=1:length(represent)
          if n == represent(p)
             switch(p)
                case {3}
                  current_color = [current_color '--'];
                case{2}
                    current_color = [current_color '-.'];
                  case{1}
                      current_color = [current_color ':']
                      
             end
                  
          end
      end
      
      
      % Regular plot
      
      if (dimen == 3) 
        plot3(plan(:,1), plan(:,2), plan(:,3), current_color, 'Linewidth',2);
      end
      if (dimen == 2)
              plot(plan(:,1), plan(:,2), current_color, 'Linewidth',2);
      end
      
      
      % Represent the goal waypoint, marking it with an x
      %color_x = [color(index) 'x'];
      %plot3 (win}(2,1), wi{n}(2,2), wi{n}(2,3), color_x, 'MarkerSize', 20);
  
  end
  
end
%axis([3 11 3 11 0 3]);
title('Evolution of the flight plans','fontsize',16,'fontweight','b');
xlabel('X(m)', 'fontsize',16);
ylabel('Y(m)','fontsize', 16);

if (dimen == 3) 
    zlabel('Z(m)','fontsize', 16);
end
set(gca, 'FontSize', 14);


for i=1:length(wps{n}(:,1,1))
    text_uav{i} = ['UAV ' num2str(i)];
end
%legend_handle=legend(text_uav);

%set(legend_handle,'fontsize',14);

hold off;


end
