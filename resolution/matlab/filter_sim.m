function filtered=filter_sim(raw_data, init_time)

curr_time = init_time;
filtered = zeros(100, 16);
curr_count=1;
for i=1:length(raw_data)
    if abs(raw_data(i, 1) - curr_time) < 1.0 && raw_data(i, 1) ~= curr_time
        curr_time = raw_data(i, 1)
        for j=1:length(raw_data(1,:))
            filtered(curr_count, j) = raw_data(i, j);
        end
        curr_count=curr_count + 1;
    end
end


end