function [A,B,C]=Linear_Matrix_Maker(Q1,Q2,Q3,Q4,Q5,Q6,Q7)


%% System Parameters
mc=7.67;
mw=0.5;
d=0;
b=0.163;
r=0.0825;
Ic=0.256;
Iw=0.02;
Im=0.01;

%% Deriving the Equations
syms x y phi teta1 teta2 teta_dot1 teta_dot2 u1 u2
U=[u1;u2];
c=r/(2*b);
I=Ic+2*mw*(d^2+b^2)+2*Im;
m=mc+2*mw;
S=[c*(b*cos(phi)-d*sin(phi)) c*(b*cos(phi)+d*sin(phi));c*(b*sin(phi)+d*cos(phi)) c*(b*sin(phi)-d*cos(phi));c -c;1 0;0 1];
M=[m 0 -mc*d*sin(phi) 0 0;0 m mc*d*cos(phi) 0 0;-mc*d*sin(phi) mc*d*cos(phi) I 0 0;0 0 0 Iw 0;0 0 0 0 Iw];
C=[-mc*d*(S(3,:)*[teta_dot1;teta_dot2])^2*cos(phi);-mc*d*(S(3,:)*[teta_dot1;teta_dot2])^2*sin(phi);0;0;0];
S_dot=(S(3,:)*[teta_dot1;teta_dot2])*[c*(-b*sin(phi)-d*cos(phi)),c*(-b*sin(phi)+d*cos(phi));c*(b*cos(phi)-d*sin(phi)),c*(b*cos(phi)+d*sin(phi));0,0;0,0;0,0];
v=[teta_dot1;teta_dot2];
f2=(S'*M*S)\(-S'*M*S_dot*v-S'*C);
Z=zeros(5,2);
dydt=[S*v;f2]+[Z;inv(S'*M*S)]*U;

%% Calculating the A,B,C Matrices
q=[x y phi teta1 teta2 teta_dot1 teta_dot2];
Q=[Q1,Q2,Q3,Q4,Q5,Q6,Q7]; % Reference Points
for i=1:7
    for j=1:7
        A_matrix(i,j)=vpa(simplify(diff(dydt(i),q(j))),3);
    end
end
for i=1:7
    for j=1:2
        B_matrix(i,j)=vpa(simplify(diff(dydt(i),U(j))),3);
    end
end
A_matrix_num=subs(A_matrix,q,Q);
B_matrix_num=subs(B_matrix,q,Q);
C_matrix_num=[1 0 0 0 0;0 1 0 0 0]; 

%% converting symbolic to double
A=double(A_matrix_num);
A(:,4:5)=[];
A(4:5,:)=[];
B=double(B_matrix_num);
B(4:5,:)=[];
C=double(C_matrix_num);
end




