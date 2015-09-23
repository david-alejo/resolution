function reduced=reduce_points(input, time_sample)

  reduced=zeros(1, length(input(1,:)));
  count = 1;
  for i=1:length(input)
    if input(i,1)/time_sample - floor(input(i,1)/time_sample) < 0.00001
%        input(i,1)/time_sample- round(input(i,1)/time_sample)
      for j=1:length(input(1,:))
	reduced(count,j) = input(i, j);
      end
      count=count+1;
    end
  end
  %Include the last point
  reduced(count,:) = input(length(input), :);
end
