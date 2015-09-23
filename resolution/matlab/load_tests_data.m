function data=load_tests_data(n_uav_init,n_uav_end,n_test_init,n_test_end)
filename='Evolution';
data=cell(n_uav_end,n_test_end - n_test_init + 1, 3);
for i=n_uav_init:n_uav_end
    for j=n_test_init:n_test_end
        [d1 d2 d3]=ddd(i,j);
        data{i}{j}{1}=d1;
        data{i}{j}{2}=d2;
        %         data{i}{j}{3}=d3;
    end
    i
end

    function [d1 d2 d3]=ddd(i,j)
        d3=[];
        directory=['test_' num2str(i,'%02.0f') '_uavs/test' num2str(j,'%04.0f')];
        evalin('base',['cd ' directory]); 
        evalin('base',filename);
        d1 = evalin('base','cost');
        d2 = evalin('base','execution_time'); 
%         d3 = evalin('base','plan');
        
        evalin('base','cd ..');
        evalin('base','cd ..');
        evalin('base','clear cost');
        evalin('base','clear execution_time');
        evalin('base','clear plan');
        evalin('base','clear global');
        evalin('base','clear classes');
    end

end