function [xdesire,ydesire]= trajectory(t,path_num)
%% Trajectory (1)
% xdesire=2*t/30;
% ydesire=sin(t/15);


%% Trajectory (2)
% xdesire=5*sin(t/100);
% ydesire=4*sin(t/50);

%% point to point
% point=[5,5];
%
% xdesire=zeros(length(t),1);
% ydesire=zeros(length(t),1);
% for i=1:length(t)
%     if t(i)<10
%         xdesire(i)=point(1)*t(i)/10;
%         ydesire(i)=point(2)*t(i)/10;
%     else
%         xdesire(i)=point(1);
%         ydesire(i)=point(2);
%     end
% end
%% Any Path
load('Data\path.mat');
if t<path_n(path_num).t(end)
    xdesire=interp1(path_n(path_num).t,path_n(path_num).x,t);
    ydesire=interp1(path_n(path_num).t,path_n(path_num).y,t);
else
    xdesire=path_n(path_num).x(end);
    ydesire=path_n(path_num).y(end);
end

end