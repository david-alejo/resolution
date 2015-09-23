function ret=loadCosts(n)

for i=1:n
    folder = ['res' num2str(i)];
    cd (folder)
    Evolution;
    ret(i,:) = cost;
    cd ..
end