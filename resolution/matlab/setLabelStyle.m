% Sets the style of one figure
function setLabelStyle(xlab, ylab, zlab)
  xlabel(xlab, 'fontsize',24);
  if (nargin > 1) 
    ylabel(ylab, 'fontsize', 24);
  end
  if (nargin > 2) 
      zlabel(zlab, 'fontsize', 24);
  end
      
  set(gca, 'FontSize', 24);
end