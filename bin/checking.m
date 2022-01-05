function [x,y] = checking(A,a,b,X,Y);
% Usable to mobile robot navigation problem
% b
% ^
% | A(a,b)
%
% A13 A23 A33
% A12 X A32
% A11 A21 A31 -> a
A11 = 50000;
A12 = 50000;
A13 = 50000;
A21 = 50000;
A23 = 50000;
A31 = 50000;
A32 = 50000;
A33 = 50000;
if (a>=2)&(b>=2)&(a<X)&(b<Y)
A11 = A(a-1,b-1);
A21 = A(a,b-1);
A31 = A(a+1,b-1);
A12 = A(a-1,b);
A32 = A(a+1,b);
A13 = A(a-1,b+1);
A23 = A(a,b+1);
A33 = A(a+1,b+1);
elseif (a>=2)&(b>=2)&(a<X) % eliminates b+1
A11 = A(a-1,b-1);
A21 = A(a,b-1);
A31 = A(a+1,b-1);
A12 = A(a-1,b);
A32 = A(a+1,b);
elseif (a>=2)&(a<X)&(b<Y) % eliminates b-1
A12 = A(a-1,b);
A32 = A(a+1,b);
A13 = A(a-1,b+1);
A23 = A(a,b+1);
A33 = A(a+1,b+1);
elseif (a>=2)&(b>=2)&(b<Y) % eliminates a+1
A11 = A(a-1,b-1);
A21 = A(a,b-1);
A12 = A(a-1,b);
A13 = A(a-1,b+1);
A23 = A(a,b+1);
elseif (b>=2)&(a<X)&(b<Y) % eliminates a-1
A21 = A(a,b-1);
A31 = A(a+1,b-1);
A32 = A(a+1,b);
A23 = A(a,b+1);
A33 = A(a+1,b+1);
elseif (a==1)&(b==1)
A32 = A(a+1,b);
A23 = A(a,b+1);
A33 = A(a+1,b+1);
elseif (a==X)&(b==1)
A12 = A(a-1,b);
A13 = A(a-1,b+1);
A23 = A(a,b+1);
elseif (a==1)&(b==Y)
A21 = A(a,b-1);
A31 = A(a+1,b-1);
A32 = A(a+1,b);
elseif (a==X)&(b==Y)
A11 = A(a-1,b-1);
A21 = A(a,b-1);
A12 = A(a-1,b);
elseif (a==1)
A21 = A(a,b-1);
A23 = A(a,b+1);
A31 = A(a+1,b-1);
A32 = A(a+1,b);
A33 = A(a+1,b+1);
elseif (a==X)
A21 = A(a,b-1);
A23 = A(a,b+1);
A11 = A(a-1,b-1);
A12 = A(a-1,b);
A13 = A(a-1,b+1);
elseif (b==1)
A12 = A(a-1,b);
A13 = A(a-1,b+1);
A23 = A(a,b+1);
A32 = A(a+1,b);
A33 = A(a+1,b+1);
elseif (b==Y)
A11 = A(a-1,b-1);
A21 = A(a,b-1);
A31 = A(a+1,b-1);
A12 = A(a-1,b);
A32 = A(a+1,b);
end
% A13 A23 A33
% A12 X A32
% A11 A21 A31 =>a
if (A11<A21)&(A11<A31)&(A11<A12)&(A11<A32)&(A11<A13)&(A11<A23)&(A11<A33)
x=a-1;
y=b-1;
elseif (A21<A31)&(A21<A12)&(A21<A32)&(A21<A13)&(A21<A23)&(A21<A33)
x=a;
y=b-1;
elseif (A31<A12)&(A31<A32)&(A31<A13)&(A31<A23)&(A31<A33)
x=a+1;
y=b-1;
elseif (A12<A32)&(A12<A13)&(A12<A23)&(A12<A33)
x=a-1;
y=b;
elseif (A32<A13)&(A32<A23)&(A32<A33)
x=a+1;
y=b;
elseif (A13<A23)&(A13<A33)
x=a-1;
y=b+1;
elseif (A23<A33)
x=a;
y=b+1;
else
x=a+1;
y=b+1;
end