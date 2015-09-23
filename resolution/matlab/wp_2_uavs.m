% Waypoint generator 2 UAVs problem

function [WP1, WP2] = wp_2_uavs(center, radius, angle)

WP1 = [0 0; 0 0];
WP1(1) = center - [radius 0];
WP1(2) = center + [radius 0];

despl = [radius*cos(angle) radius*sin(angle)]

WP2(1) = center + despl;
WP2(2) = center - despl

end