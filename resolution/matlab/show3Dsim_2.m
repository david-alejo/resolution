
function show3Dsim(data,geom)
%Shows a 3D view of the trajectories within 'data'
%'data' must be loaded calling soltraj.m
%'geom' is loaded from geometry.in
% w / x : accelerate/deccelerate
% d / a : frame forward/backwards
% s : changes to pause/play mode (in pause mode all buttons are usable)
% r : rewinds to start time
% q : exits animation
%
% NOTE: if the 3D camera orientation button is used, keyboard will not
% function until the button is "unlocked" (press it again)
%


semigeom= 0.5*geom;
n_uavs= length(data);
time_lap_base=0.2;
time_lap=time_lap_base;
color_cm={'y';'b'; 'r'; 'g'; [0.6 0.6 0.6]; };
%Initializing boxes
h=figure('KeyPressFcn',@changeSpeed);
cajas=cell(n_uavs,size(geom,1));
for n=1:n_uavs
    for k=1:size(geom,1)
        [X Y Z]=boxFromLoHi(data{n}(1,1)-semigeom(k,1)+geom(k,4), ...
                        data{n}(1,2)-semigeom(k,2)+geom(k,5), ...
                        data{n}(1,3)-semigeom(k,3)+geom(k,6), ...
                        data{n}(1,1)+semigeom(k,1)+geom(k,4), ...
                        data{n}(1,2)+semigeom(k,2)+geom(k,5), ...
                        data{n}(1,3)+semigeom(k,3)+geom(k,6));
        cajas{n}{k}=patch(X, Y, Z, color_cm{mod(n,length(color_cm))+1});
    end
end


axis([-1, 16, -1, 16, -1, 3.0, -1, 1]);
set(gca,'PlotBoxAspectRatio' ,[1 1 1 ]);
set(gca,'DataAspectRatio' ,[1 1 1 ]);
set(gca,'CameraPosition',[-64 -64 64]);
set(gca,'CameraTarget',[7.5 7.5 1.5]);
set(gca,'XGrid','on');
set(gca,'YGrid','on');
set(gca,'ZGrid','on');
h = gcf;


% stopSim=0;
% stopBox=zeros(n_uavs);
t=1;
pause_mode=0;
quit_prog=0;
while ~quit_prog
    drawBoxes(t);
    if pause_mode==0
        t=t+1;
    
    end
    pause(time_lap);
end
return;
    function drawBoxes(c_time)
        for nn=1:n_uavs
            if c_time <=size(data{nn},1)
                for kk=1:size(geom,1)
                    [X Y Z]=boxFromLoHi(data{nn}(c_time,1)-semigeom(kk,1)+geom(kk,4), ...
                        data{nn}(c_time,2)-semigeom(kk,2)+geom(kk,5), ...
                        data{nn}(c_time,3)-semigeom(kk,3)+geom(kk,6), ...
                        data{nn}(c_time,1)+semigeom(kk,1)+geom(kk,4), ...
                        data{nn}(c_time,2)+semigeom(kk,2)+geom(kk,5), ...
                        data{nn}(c_time,3)+semigeom(kk,3)+geom(kk,6));
                    set(cajas{nn}{kk},'XData',X);
                    set(cajas{nn}{kk},'YData',Y);
                    set(cajas{nn}{kk},'ZData',Z);
                end
%             else
%                 stopBox(nn)=1;
            end
        end
%         if all(stopBox)
%             stopSim=1;
%         end
        drawnow;
    end

    function changeSpeed(src,evnt)
        if evnt.Character == 'w'
            time_lap= time_lap - 0.01;
        elseif evnt.Key == 'x' && time_lap > 0.0
            time_lap= time_lap + 0.01;
        elseif evnt.Key == 's'
            if pause_mode==1
                pause_mode=0;
            else
                pause_mode =1;
            end
        elseif evnt.Key == 'r'
            t=1;
            drawBoxes(t);
        elseif evnt.Key == 'd'
%             if stopSim==0
                t=t+1;
                drawBoxes(t);
%             end
        elseif evnt.Key == 'a'
            if t>1
                t=t-1;
                drawBoxes(t);
            end
        elseif evnt.Key == 'q'
            quit_prog=1;
        end
        
    end
    function [X Y Z]=boxFromLoHi(lx,ly,lz,hx,hy,hz)
        %Remember that each column (not each row) is a rectangle, the order
        %is:
        %Front, top, right, bottom, left, back
        X=[lx lx hx lx lx lx; lx lx hx lx lx hx; hx hx hx hx lx hx; hx hx hx hx lx lx];
        Y=[ly ly ly ly ly hy; ly hy hy hy hy hy; ly hy hy hy hy hy; ly ly ly ly ly hy];
        Z=[lz hz hz lz hz hz; hz hz hz lz hz hz; hz hz lz lz lz lz; lz hz lz lz lz lz];
        
    end


end