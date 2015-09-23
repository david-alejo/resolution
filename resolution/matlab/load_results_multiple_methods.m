% Loads the test obtained with the resolution files
% prefix --> a cell with the start of the name
% uavs --> a vector with the min and max UAVs (in order)
function results=load_results_multiple_methods(prefix, uavs)
  results=cell(length(prefix), 1);

  for i = 1:length(prefix);
    results{i} = load_results(prefix{i}, uavs);
  end

end