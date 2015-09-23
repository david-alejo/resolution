function generate_secuence_figures(name, n_shots, axis_, label_x, label_y)
    for i=0:n_shots
        h = figure;
        curr_name = [name num2str(i)] 
        a = load(curr_name);
        represent_swarm_shot(a, axis_);
        setLabelStyle(label_x, label_y);
        curr_name_jpg = [curr_name '.jpg']
        saveas(h, curr_name_jpg, 'jpg');
        close(h)
    end
end