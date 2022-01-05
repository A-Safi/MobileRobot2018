%% Instruction
% 1- Run Path Planning 
% 2- Run Simulation

%% Artificial Potential Field Path Planning
clc;clear;close all

%%
syms x y z
newmap= input('a new map? (default "no") (yes=1,no=2)');
m=input('number of ways (default "3"): ');
v=input('Velocity(cm/s)(default "5"): ');
r0=1.1;%input('safe distance:');
w=10;
if newmap==1
    n=input('number of obstacles:');
    %n=2;
    %w = input('points weight:');
    % w=10;
    %r0=input('safe distance:');
    %r0=10;
    % m=2;
    
    map=[];
    for i=1:n
        posf=0;
        setappdata(0,'posf',posf);
        mapf=0;
        setappdata(0,'mapf',mapf);
        
        i=1:100;
        j=1:100;
        
        setappdata(0,'map',map);
        A=zeros(100,100);
        mapf=1;
        setappdata(0,'mapf',mapf);
        
        figure(1)
        mesh(i,j,A(i,j))
        axis([1,100,1,100,-100,700])
        view([-20,-15,20])
        hold on
        
        figure(2)
        contour(i,j,A(i,j),60)
        hold on
        %h=msgbox('Draw a wall by using the left mouse button for start and right mouse button for ending the wall');
        %uiwait(h,5);
        %if ishandle(h)==1
        %            delete(h);
        %end
        %but=0;
        %while(but~=1)
        %     [xs,ys,but]=ginput(1);
        % end
        %but=0;
        %while(but~=3)
        %    [xe,ye,but]=ginput(1);
        %end
        xxs=input('Xcenter of obstacles:');
        yys=input('Ycenter of obstacles:');
        D=input('diameter of obstacles:');
        xe=xxs;%+D/2;
        ye=yys;%+D/2;
        xs=xxs;%-D/2;
        ys=yys;%-D/2;
        xss=xs/10;%round(xs*10)/100;
        yss=ys/10;%round(ys*10)/100;
        xee=xe/10;%round(xe*10)/100;
        yee=ye/10;%round(ye*10)/100;
        w=w(1);
        map = getappdata(0,'map');
        map=fmap([xss,yss],D,map,w);
        setappdata(0,'map',map);
    end
    i=1:100;
    j=1:100;
    
    x=map(:,2);
    y=map(:,1);
    w=map(:,3);
    R=[];
    
    A1=[];
    for I=1:100
        for J=1:100
            R=[];
            for r=1:length(x)-1
                % R=[R w(r)/((J/10-y(r))^2+(I/10-x(r))^2)];
                ro(r)=sqrt((J/10-y(r))^2+(I/10-x(r))^2);
                %r0=5;
                
                if ro(r)<r0
                    R=[R w(r)*(1/ro(r)-1/r0)*1/(ro(r)^2)];
                else
                    R=[R 0];
                end
            end
            %          RG = sqrt((J/10-GoalY)^2+(I/10-GoalX)^2);
            A1(I,J)= sum(R);
            if (A1(I,J)>700)
                A1(I,J)=700;
            end
        end
    end
    A=A1;
    figure(1)
    mesh(i,j,A(i,j))
    axis([1,100,1,100,-100,700])
    view([-20,-15,20])
    hold on
    
    figure(2)
    contour(i,j,A(i,j),60)
    hold on
    
    
else
    posf=0;
    setappdata(0,'posf',posf);
    mapf=1;
    setappdata(0,'mapf',mapf);
    
    i=1:100;
    j=1:100;
    w=10;%input('points weight:');
    map1=[];
    D=12;
    
    b(1,:)=[2,2];
    b(2,:)=[5,5];
    b(3,:)=[5,2];
    b(4,:)=[2,5];
    b(5,:)=[6,8];
    b(6,:)=[8,6];
    b(7,:)=[4,2];
 for jjj=1:6   
    map1=fmap(b(jjj,:),D,map1,w);
 end
 
    map=map1;
    setappdata(0,'map',map1);
    x=map(:,2);
    y=map(:,1);
    w=map(:,3);
    R=[];
    A1=[];
    
    for I=1:100
        for J=1:100
            R=[];
            for r=1:length(x)-1
                R=[R w(r)/((J/10-y(r))^2+(I/10-x(r))^2)];
            end
            A1(I,J) = sum(R);
            if (A1(I,J)>700)
                A1(I,J)=700;
            end
        end
    end
    A=A1;
    figure(1)
    mesh(i,j,A(i,j))
    axis([1,100,1,100,-100,700])
    view([-20,-15,20])
    hold on
    figure(2)
    contour(i,j,A(i,j),60);axis equal;
    hold on
end


%i=1:100;
%j=1:100;

A=A1;
%Figure(1) % Select the proper axes
%mesh(i,j,A(i,j))
%axis([1,100,1,100,-10,300])
%view([-20,-15,20])

%Figure(2)
%contour(i,j,A(i,j),60)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w =w(1);
%h=msgbox('Select the target using the left mouse button');
%uiwait(h,5);
%if ishandle(h)==1
%    delete(h);
%end
%but=0;
%while(but~=1)
%    [xval,yval,but]=ginput(1);
%end
xval=90;%input('target x:');
yval=90;%input('target y:');


pos=[xval/10,yval/10];
setappdata(0,'pos',pos);
posf=1;
setappdata(0,'posf',posf);

%hold on
% figure(2)
% plot(xval,yval,'ro');
map = getappdata(0,'map');
pos = getappdata(0,'pos');
posf = getappdata(0,'posf');
xval=round(pos(1)*10)/10;
yval=round(pos(2)*10)/10;
map=[map;
    xval  yval   w];
save('Data\Map.mat','A');
path=struct;
for mm=1:m
    [A,B,LMF,NoSol,xx,yy]=getpath(map,r0,mm,m);
    path(mm).x=xx;
    path(mm).y=yy;
    %i=1:100;
    %j=1:100;
    
    %if(LMF==1)
    %    display('Local Minimum exists')
    %%set(handles.text2,'String','Local Minimum exists')
    %end
    
    if(NoSol==1)
        display('No Solution')
        %set(handles.text2,'String','No Solution has been found')
    else
        if(LMF==1)
            display('Local Minimum exists')
            %set(handles.text2,'String','Local Minimum exists')
        else
            display('A Solution exist')
            %  set(handles.text2,'String','Path has been found')
        end
    end
    if mm==1
        figure(1)
        hold off% Select the proper axes
        mesh(i,j,A(i,j)+B(i,j))
        axis([1,100,1,100,-100,10000])
        view([-20,-15,20])
        %set(handles.axes1,'XMinorTick','on')
        hold off
        figure(2)
        %axes(handles.axes2) % Select the proper axes
        contour(i,j,A(i,j)+B(i,j),60)
        %set(handles.axes2,'XMinorTick','on')
    end
    
    posf=0;
    setappdata(0,'posf',posf);
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% PathProcessing %%%%%%%%%%%%%%%%%%%%%%%
k=1;
%% maping coef
nn=20;
%%
xval=xval*10/nn;
yval=yval*10/nn;

for j=1:length(path)
    y=path(j).x/nn;
    x=path(j).y/nn;
    
    n=length(x);
    
    x_new=zeros(1,n);
    y_new=zeros(1,n);
    distance=zeros(n);
    total_distance=0;
    point=1;
    
    for i=1:n
        for j=1:n
            distance(i,j)=sqrt((x(i)-x(j)).^2+(y(i)-y(j)).^2);
        end
    end
    maxx=max(max(distance))+1;
    distance_modified=distance+maxx*eye(n);
    distance_modified(:,point)=maxx;
    for i=1:n
        x_new(i)=x(point);
        y_new(i)=y(point);
        if i~= n
            [val,point]=min(distance_modified(point,:));
            total_distance=total_distance + val ;
            distance_modified(:,point)=maxx;
        end
    end
    
    %% smoothing data
    if abs(x_new(end)-xval)<2/nn && abs(y_new(end)-yval)<2/nn
        Time_end=total_distance/(v*1e-2);
        time=linspace(0,Time_end,n);
        sprintf('Total distance : %5.2f m\nTime : %5.2f sec',total_distance,Time_end)
        
        y_n=smooth(time,y_new,10);
        x_n=smooth(time,x_new,10);
        path_n(k).d=total_distance;
        path_n(k).x=x_n-x_n(1);
        path_n(k).y=y_n-y_n(1);
        path_n(k).t=time;
        k=k+1;
    end
end
%% Plots
if exist('path_n')==0
    error('No Solution Exist')
end
figure(3)
plot(path_n(1).x(1),path_n(1).y(1),'sk','MarkerSize',10);hold on
plot(path_n(1).x(end),path_n(1).y(end),'ok','MarkerSize',10)
load('Data\Map.mat','A');
contour(1/nn:1/nn:100/nn,1/nn:1/nn:100/nn,A);axis equal;
title('Path Planning')
xlabel('x(meter)')
ylabel('y(meter)')
for i=1:length(path_n)
    plot(path_n(i).x,path_n(i).y,'LineWidth' , 1.5);
end
legend('start point','end point','Location','SouthEast');
save('Data\Path.mat','path_n','nn');








