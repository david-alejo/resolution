% Loads the test obtained with the resolution files
function results=load_results(prefix, uavs)
% prefix --> a cell with the start of the name
%uavs --> a vector with the min and max UAVs (in order)

results=cell(uavs(2),1);

for i=uavs(1):uavs(2)
  name = strcat(prefix, num2str(i));
  eval(name);
  results{i}=cell(2,1);
  
  % Get the non-zero results
  non_zero = 0;
  for j=1:length(result)
    if (result(j,1) > 0) 
      non_zero=non_zero+1;
    end
  end
  aux_cost = cell(non_zero,1);
  aux_time = cell(non_zero,1);
  j_aux = 1;
  for j=1:length(result)
    if (result(j,1) > 0) 
      aux_cost{j_aux}=evo_cost{j};
      aux_time{j_aux}=evo_time{j};
      j_aux=j_aux+1;
    end
  end
  
  
  results{i}{1}=aux_cost;
  results{i}{2}=aux_time;

end