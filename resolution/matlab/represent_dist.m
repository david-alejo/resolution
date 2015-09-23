function represent_dist(time, dist, mindist)
    figure;
    hold on;
    dist_xy = sqrt(dist(:,2).^2 + dist(:,3).^2) + 0.15;
    plot (time, abs(dist(1:length(time), 4)), 'b', 'LineWidth', 4)
    plot (time, dist_xy(1:length(time)), 'r', 'LineWidth', 4);
    setStyle('Time (s)', 'Separation(s)');
    aux1 = [time(1) time(length(time))];
    plot (aux1, [mindist(1) mindist(1)], 'r--', 'LineWidth', 3);
    plot (aux1, [mindist(2) mindist(2)], 'b--', 'LineWidth', 3);
end