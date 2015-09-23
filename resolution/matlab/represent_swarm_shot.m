function represent_swarm_shot(shot, axis_)
    hold on;
    for i=1:length(shot)
        plot(shot(i,1), shot(i,2), 'o', 'MarkerSize', 8);
        
    end

    axis(axis_);
    title('Coordinates of the intermediate waypoint');
end