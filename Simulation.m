%% Advance Dynamics Project 2018
% Path Planning, Controller Design and Simulation
% for a nonlinear Mobile-Robot 
% 1- Run Path Planning 
% 2- Run Simulation

%%
clear;clc;close all

%% Add Directory
ds=genpath(pwd);
addpath(ds);

%% Linearization
[A,B,C]=Linear_Matrix_Maker(0,0,0,0,0,0.01,0.01);
% 
%% LQR (Feedback Gains)
Q = diag([150,150,2e-16,1,1]);
R = diag([1000,1000]);

K = lqr(A,B,Q,R);
save('Data\Kmat.mat','K')

%% Simulation
load('Data\path.mat'); % Loading All Pathes

sol=struct; % Solution Structure 

% Defining Cost Matrix for All the Solutions
total_cost=zeros(length(path_n),1);

% Solving for all Pathes, Inside a Loop
for path_num=1:length(path_n)
    %% Solving Process
    
    tspan=[0 path_n(path_num).t(end)+20;];
    initialc=[0 0 0 0 0 0 0 0 0 0];
    y0=initialc;
    options = odeset('RelTol',1e-3);
    [t,y]=ode45(@(t,y) odefunc(t,y,path_num) ,tspan,y0,options);
    
    %% Trajectory Tracking
    xdesire=zeros(length(t),1);
    ydesire=zeros(length(t),1);
    for i=1:length(t)
        [xdesire(i) ,ydesire(i)]=trajectory(t(i),path_num);
    end
    
    %% Control effort
    inputmat=[0;0];
    save('Data\input_mat.mat','inputmat');
    u=zeros(2,length(t));
    for i=1:length(t)
        u(:,i)=control_function(y(i,:),t(i),path_num);
    end
    
    %% Computations for Cost function and Making Them Dimensionless
    
    n=length(t);
    
    % Phi dot 
    phi_d=zeros(n,1);
    phi_d(2:end,:)=diff(y(:,3));
    phi_d=phi_d/max(phi_d);
    
    % First Input
    u1=u(1,:)';
    u1c=u1.^2/max(u1.^2);
    
    % Second Input
    u2=u(2,:)';
    u2c=u2.^2/max(u2.^2);
    
    % Error 
    ex=y(:,1)-xdesire(end);
    ey=y(:,2)-ydesire(end);
    exc=ex.^2/max(ex.^2);
    eyc=ey.^2/max(ey.^2);
    
    cost=(u1c+u2c+exc+eyc);
    
    % Saving Solutions Inside The Solution Structure
    sol(path_num).t=t;
    sol(path_num).u=u';
    sol(path_num).y=y;
    sol(path_num).cost=cost;
    total_cost(path_num)=trapz(t,cost);
end

%% Remove Directory
rmpath(ds);

% Finding Which Path Has the Minimum Cost
[valmin,best_path]=min(total_cost);

%%  plot Results (For Minimum-Cost Path)
b=0.163;r=0.0825;
figure(1);hold on
load('Data\Map.mat','A')
plot(path_n(best_path).x,path_n(best_path).y,'-b', 'LineWidth' , 2.5);
plot(sol(best_path).y(:,1),sol(best_path).y(:,2),'r', 'LineWidth' , 1.5); axis equal;grid on;
contour(1/nn:1/nn:100/nn,1/nn:1/nn:100/nn,A);
title('Trajectory Tracking');
axis([min(path_n(best_path).x)-1 max(path_n(best_path).x)+1 ...
    min(path_n(best_path).y)-1 max(path_n(best_path).y)+1]);
ylim([0 5]);xlim([0 5]);

xlabel('x(meter)');ylabel('y(meter)');

figure(2)
plot(sol(best_path).t,sol(best_path).cost,'r', 'LineWidth' , 2);hold on
title('Cost');
xlabel('Time(seconds)');ylabel('Cost');

figure(3)
plot(sol(best_path).t,sol(best_path).u(:,1),'r', 'LineWidth' , 2);hold on
plot(sol(best_path).t,sol(best_path).u(:,2),'b', 'LineWidth' , 2);grid on
title('Control effort');
xlabel('Time(seconds)');ylabel('Inputs(N.m)');
legend('Rigth Motor','Left Motor');

%% States
figure(4)
subplot(3,2,1)
plot(sol(best_path).t,sol(best_path).y(:,1),'r', 'LineWidth' , 2);grid on;hold on;
plot(path_n(best_path).t,path_n(best_path).x,'--b', 'LineWidth' , 1.5);
title('State 1')
xlabel('t(sec)');ylabel('x(m)');
legend('Actual','Desired');
subplot(3,2,2)
plot(sol(best_path).t,sol(best_path).y(:,2),'r', 'LineWidth' , 2);grid on;hold on;
plot(path_n(best_path).t,path_n(best_path).y,'--b', 'LineWidth' , 1.5);
title('State 2')
xlabel('t(sec)');ylabel('y(m)');
legend('Actual','Desired');
subplot(3,2,3)
plot(sol(best_path).t,sol(best_path).y(:,3),'r', 'LineWidth' , 2);
xlabel('t(sec)');ylabel('Phi(rad)');grid on;
title('State 3')
subplot(3,2,4)
plot(sol(best_path).t,sol(best_path).y(:,6),'r', 'LineWidth' , 2);
title('State 6')
xlabel('t(sec)');ylabel('theta-dot rigth (rad/s)');grid on;
subplot(3,2,5)
plot(sol(best_path).t,sol(best_path).y(:,7),'r', 'LineWidth' , 2);
title('State 7')
xlabel('t(sec)');ylabel('theta-dot left (rad/s)');grid on;
subplot(3,2,6)
plot(sol(best_path).t,(sol(best_path).y(:,7)+sol(best_path).y(:,6))/2*r*100,'g', 'LineWidth' , 2);
title('Velocity of the vehicle')
xlabel('t(sec)');ylabel('Velocity (cm/s)');grid on;
%%
subplot(3,1,1)
plot(sol(best_path).t,sol(best_path).y(:,8),'r', 'LineWidth' , 2);
title('State 8')
xlabel('t(sec)');ylabel('');grid on;
subplot(3,1,2)
plot(sol(best_path).t,sol(best_path).y(:,9),'r', 'LineWidth' , 2);
title('State 9')
xlabel('t(sec)');ylabel('');grid on;
subplot(3,1,3)
plot(sol(best_path).t,sol(best_path).y(:,10),'r', 'LineWidth' , 2);
title('State 10')
xlabel('t(sec)');ylabel('');grid on;
%% Animation
answer = questdlg('Would you like to see the animation?', ...
    'Animation', ...
    'Yes','No','No');
switch answer
    case 'Yes'
        question = 1;
    case 'No'
        question = 0;
end
if question==1
    figure(1);
    plot(path_n(best_path).x,path_n(best_path).y,'-b', 'LineWidth' , 2);
    plot(sol(best_path).y(:,1),sol(best_path).y(:,2),'r', 'LineWidth' , 1.5); axis equal;hold on;grid on;
    crank=[];
    for i=1:5:length(sol(best_path).y)
        figure(1);
        p1=[-2*b*sin(sol(best_path).y(i,3)/b)+sol(best_path).y(i,1) 2*b*cos(sol(best_path).y(i,3)/b)+sol(best_path).y(i,2)];
        p2=[2*b*sin(sol(best_path).y(i,3)/b)+sol(best_path).y(i,1) -2*b*cos(sol(best_path).y(i,3)/b)+sol(best_path).y(i,2)];
        delete(crank);
        crank=line([p1(1) p2(1)],[p1(2) p2(2)],'Color','g','LineWidth',5);
        axis(gca,'equal');
        axis([min(sol(best_path).y(:,1))-1 max(sol(best_path).y(:,1))+1 ...
            min(sol(best_path).y(:,2))-1 max(sol(best_path).y(:,2))+1]);
        drawnow
        title(['Time : ',num2str(sol(path_num).t(i)),' Seconds']);
        xlabel('x(meter)');ylabel('y(meter)');
    end
end