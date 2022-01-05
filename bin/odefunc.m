function dydt = odefunc(t,y,u1,u2,tspan)
%% System Parameters
mc=7.67;
mw=0.5;
d=0.01;
b=0.163;
r=0.0825;
c=r/(2*b);
e=0;
Ic=0.256;
m=mc+2*mw;
Iw=0.02;
Im=0.01;
I=Ic+2*mw*(d^2+b^2)+2*Im;
D=0.4;
C_w=1;
L1=[0 1 0;-1 0 0];
L2=[0 0 1;-1 0 0];

%% Solving Process
dydt=zeros(10,1);
A=[-sin(y(3)/b) cos(y(3)/b) -d 0 0;-cos(y(3)/b) -sin(y(3)/b) -b r 0;-cos(y(3)/b) -sin(y(3)/b) b 0 r];
AT=transpose(A);
B=transpose([0 0 0 1/r 0;0 0 0 0 1/r]);
S=[c*(b*cos(y(3)/b)-d*sin(y(3)/b)) c*(b*cos(y(3)/b)+d*sin(y(3)/b));c*(b*sin(y(3)/b)+d*cos(y(3)/b)) c*(b*sin(y(3)/b)-d*cos(y(3)/b));c -c;1 0;0 1];
M=[m 0 -mc*d*sin(y(3)/b) 0 0;0 m mc*d*cos(y(3)/b) 0 0;-mc*d*sin(y(3)/b) mc*d*cos(y(3)/b) I 0 0;0 0 0 Iw 0;0 0 0 0 Iw];
C=[-mc*d*(1/b^2)*(S(3,:)*[y(6);y(7)]+AT(3,:)*e*[y(8);y(9);y(10)])^2*cos(y(3)/b);-mc*d*(1/b^2)*(S(3,:)*[y(6);y(7)]+AT(3,:)*e*[y(8);y(9);y(10)])^2*sin(y(3)/b);0;0;0];
S_dot=(1/b)*(S(3,:)*[y(6);y(7)]+AT(3,:)*e*[y(8);y(9);y(10)])*[c*(-b*sin(y(3)/b)-d*cos(y(3)/b)),c*(-b*sin(y(3)/b)+d*cos(y(3)/b));c*(b*cos(y(3)/b)-d*sin(y(3)/b)),c*(b*cos(y(3)/b)+d*sin(y(3)/b));0,0;0,0;0,0];
AT_dot=transpose((1/b)*(S(3,:)*[y(6);y(7)]+AT(3,:)*e*[y(8);y(9);y(10)])*[-cos(y(3)/b) -sin(y(3)/b) 0 0 0;sin(y(3)/b) -cos(y(3)/b) 0 0 0;sin(y(3)/b) -cos(y(3)/b) 0 0 0]);
H=[transpose(S)*M*S transpose(S)*M*transpose(A);A*M*S A*M*transpose(A)];
if sqrt(y(8)^2+(y(9)+r*y(6))^2)<10^(-6)
    v1=10^(-6);
else
v1=sqrt(y(8)^2+(y(9)+r*y(6))^2);
end
if sqrt(y(8)^2+(y(10)+r*y(7))^2)<10^(-6)
    v2=10^(-6);
else
v2=sqrt(y(8)^2+(y(10)+r*y(7))^2);
end
FF=(1/v1)*transpose(L1)*([C_w 0;0 D])*L1+(1/v2)*transpose(L2)*([C_w 0;0 D])*L2;
F=-AT*FF*A*(S*[y(6);y(7)]+AT*e*[y(8);y(9);y(10)]);
P=([S transpose(A)])\(-(S_dot*[y(6);y(7)]+AT_dot*e*[y(8);y(9);y(10)])-M\C)+H\([0;0;A*F]);
R=([S transpose(A)])\(M\B);
dydt(1:5)=S*[y(6);y(7)]+AT*e*[y(8);y(9);y(10)];
u=control_function(y,t,1);
dydt(6:10)=P+R*u;
end