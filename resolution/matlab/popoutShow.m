function popoutShow(data,popsize)

iterations= size(data,1) /popsize

it_step= ceil(iterations/20)
it_length= ceil(iterations/it_step);

% k=data(1:it_length,:);
% cm{1}=data(1:iterations,:);
nn=1
for n=0:it_step:iterations-1
%     k=vertcat(k, data( (n-it_step)*popsize+1:n*popsize,:) );
    cm{nn}=data( n*popsize+1:(n+1)*popsize,:);
    nn=nn+1;
end

for n=1:it_length
    me(n)=mean(cm{n}(:,8) );
    mi(n)=min(cm{n}(:,8) );
    col(:,n)=cm{n}(:,8);
end

for m=1:size(col,1)
    for n=1:size(col,2)
        if col(m,n)==1e10
            col(m,n)=NaN;
        end
    end
end

figure('OuterPosition',[0, 0, 800, 600]);

for i=1:it_length
  subplot(4,ceil(it_length/4),i);
  hist(col(:,i),20);
%   while n(1)==0;
%       xout(1)=[];
%       n(1)=[];
%   end
%   while n(end)==0;
%       xout(end)=[];
%       n(end)=[];
%   end
%   bar(xout,n);
  h = findobj(gca,'Type','patch');
  set(h,'EdgeColor','none');

end
suplabel('Histogram of population cost function evaluation','t');
suplabel('Occurrences','y');
suplabel('Flight cost [$]');
hgsave('histogram');
print('-dpng','histogram'); 


% figure;
% plot(me);
% xlabel('average individual cost function value');
% figure;
% plot(mi);
% xlabel('minimum individual cost function calue');
figure('OuterPosition',[100, 100, 800, 600]);
boxplot(col,'symbol','','whisker',50.0);
title('Cost function value:  min|-----|p25  median  p75|-----|max')
xlabel('Iteration');
ylabel('Flight cost [$]');
hgsave('boxplot');
print('-dpng','boxplot'); 
close all;
end