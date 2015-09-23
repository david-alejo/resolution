% Waypoint generator 2 UAVs problem

function [WP] = wp_2_uavs(center, radius, uavs)

WP = cell(uavs, 1);

for i=1:uavs

  angle = i*2*pi/uavs;

  WP{i} = [0 0; 0 0];
  WP{i}(1, :) = center - [radius*cos(angle) radius*sin(angle)];
  WP{i}(2, :) = center + [radius*cos(angle) radius*sin(angle)];
  
end

end