function show_Evolution_3D_different_figures(wps, dimen, shift_m)

% This vector indicates the generations that will be represented
represent = [ 7 15 23 30];

% Setting Style. NOT USED
%set(0,'DefaultAxesColorOrder', color);
%set(0,'DefaultAxesLineStyleOrder','-|-.|--|:')


% This definition of color allows changes in the deep of the color
color = [ 1 0 0; 0 0.7 0; 0 0 1; 0 0 0];

% This definition makes easier the definition of the line style
%color = [ 'r' 'g' 'b' 'k' ];

% The first figure will be the initial flight plans
figure;
hold on;
for uav=1:length(wps{1}(:,1,1))
    %Get the first waypoint
      plan(1,:)=wps{1}(uav,1,:);
      % And the last one
      plan(2,:)=wps{1}(uav,length(wps{1}(1,:,1)),:);
      
      
        
      % Calculate the index to the current color
      index = mod(uav, length(color)) + 1;
        
      % Represent the flight plan         
      
      % Obtain the color of the current plot
      current_color = color(uav, :);
      
      represent_plan(plan, dimen, current_color);
               
  end
  
  %Represent the arrows
  for uav=1:length(wps{1}(:,1,1))
      clear plan;
      %Get the first waypoint
      plan(1,:)=wps{1}(uav,1,:);
      % And the last one
      plan(2,:)=wps{1}(uav,length(wps{1}(1,:,1)),:);
      
      % Get the color
      index = mod(uav, length(color));
      current_color = color(uav, :);
      
      
      
      represent_arrow(plan, dimen, current_color, shift_m);
  end
  % Setting the legend text
    for i=1:length(wps{1}(:,1,1))
        text_uav{i} = ['UAV ' num2str(i)];
    end
  lga = legend(text_uav);
    set(lga, 'FontSize', 16);
    set(lga, 'FontName', 'Arial');
    
    % Set the labels
    xlabel('X(m)', 'fontsize',16);
    ylabel('Y(m)','fontsize', 16);
    if (dimen == 3) 
    % If necessary, set the zlabel
    zlabel('Z(m)','fontsize', 16);
    end
    hold off;

% Now we are going to represent the flight plans obtained in the desired
% generations (that are indicated in represent vector)
for n=represent
    figure;
    hold on;
    represent_plans(wps{n}, dimen, color, shift_m);
end

hold off;
end

function represent_plans(plans, dimen, colors, shift_m)
    
    for uav=1:length(plans(:,1,1))
        hold on;
      % Calculate the index to the current color
      index = mod(uav - 1, length(colors)) + 1;                    
      % Obtain the color of the current plot
      current_color = colors(index, :);
      p(:,:) = plans(uav,:,:);
      represent_plan(p, dimen, current_color);
      hold on;
    end
    
    for uav=1:length(plans(:,1,1))
      % Calculate the index to the current color
      index = mod(uav - 1, length(colors)) + 1;                    
      % Obtain the color of the current plot
      current_color = colors(index, :);
      
      
      p(:,:) = plans(uav,:,:);
      represent_arrow(p, dimen, current_color, shift_m);
      
    end
    
    % Setting the legend text
    for i=1:length(plans(:,1,1))
        text_uav{i} = ['UAV ' num2str(i)];
    end
    lga = legend(text_uav);
    set(lga, 'FontSize', 16);
    set(lga, 'FontName', 'Arial');
    
    % Set the labels
    xlabel('X(m)', 'fontsize',16);
    ylabel('Y(m)','fontsize', 16);
    if (dimen == 3) 
    % If necessary, set the zlabel
    zlabel('Z(m)','fontsize', 16);
    hold off;
    end

    set(gca, 'FontSize', 14);
end

function represent_plan(plan, dimen, color)   
      % Regular plot
      if (dimen == 3) 
        plot3(plan(:,1), plan(:,2), plan(:,3), 'Color', color, 'Linewidth',2);        
      else       
          plot(plan(:,1), plan(:,2), 'Color', color, 'Linewidth',2);
      end
      
end

function represent_arrow(plan, dimen, color, shift_m)
  % Represent an arrow that starts in the first waypoint
  % And goes to the second. It will be shifted in the perpendicular
  % direction
      
  start = plan(1,:);
  seg = plan(2,:) - plan(1,:);
  length_ = seg * 0.2;
  if (dimen == 2) 
    shift = [ -seg(2), seg(1) ];
    shift = shift / norm(shift) * 0.5;
  else 
    shift = [ -seg(2), seg(1) 0];
    shift = shift / norm(shift) * 0.5;
  end
  shift = shift * shift_m;
      
  start = start + shift;
  edge = start + length_;
  arrow(start, edge, 'FaceColor', color, 'EdgeColor', color, 'LineWidth', 2)  
end  
