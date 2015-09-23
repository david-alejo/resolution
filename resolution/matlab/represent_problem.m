function represent_problem(start, goal, obstacle)
    hold on;
    plot(start(1), start(2), 'o', 'MarkerSize', 8);
    plot(goal(1), goal(2), 'x', 'MarkerSize', 10);
    for i =1:length(obstacle(:,1))
      circle(obstacle(i, 1), obstacle(i, 2), obstacle(i, 3));
    end
%      axis(axis_);
%      axis equal;
end