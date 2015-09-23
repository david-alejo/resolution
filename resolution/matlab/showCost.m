% This function loads the costs obtained from multiple simulations,
% calculates the mean minimum cost in each generation and represents it.
function showCost(costs)

color=['r' 'b' 'g' 'k'];

for m=1:length(costs) 
     for i=1:length(costs{m}(1,:))
        cost{m}(i) = mean(costs{m}(:,i));
    end


figure;
hold on;
%axis([3 11 3 11]);

for m=1:length(cost)
    plot(cost{m}, color(m), 'Linewidth', 2);
end
    
%title('Evolution of the minimum cost','fontsize',16,'fontweight','b');
xlabel('Generation', 'fontsize',16);
ylabel('Cost(m)','fontsize', 16);
set(gca, 'FontSize', 14);

text{1}='2D Exploration';
text{2}='3D Exploration';


legend_handle=legend(text);

set(legend_handle,'fontsize',14);

hold off;


end
