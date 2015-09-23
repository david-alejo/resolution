function uavtext = getUAVText(uav_min, uav_max, text)
   %uavtext = cell(uav_max - uav_min + 1);
   if nargin == 2
     text = 'UAV';
   end
   d=1;
   for i=uav_min:uav_max
     uavtext{d} = strcat(num2str(i),text);
     d=d+1;
   end
end