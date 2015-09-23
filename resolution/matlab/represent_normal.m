function represent_normal(mu, sigma, limits)
 %mu = [0 0];
 %Sigma = [.25 .3; .3 1];
 limits1=[-3 3;-3 3];
 if (nargin > 2) 
     limits1 = limits; 
 end
 x1 = limits1(1,1):.2:limits1(1,2)
 x2 = limits1(2,1):.2:limits1(2,2);
 [X1,X2] = meshgrid(x1,x2);
 F = mvnpdf([X1(:) X2(:)],mu,sigma);
 F = reshape(F,length(x2),length(x1));
 surf(x1,x2,F);
 caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
 axis([limits1(1,1) limits1(1,2) limits1(2,1) limits1(2,2) 0 .2])
 setLabelStyle('x','y'); zlabel('Probability Density');
end