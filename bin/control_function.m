function u=control_function(y,t,path_num)
load('Data\Kmat','K');
[xdesire, ydesire]=trajectory(t,path_num);

eX=y(1)-xdesire;
eY=y(2)-ydesire;
e=[cos(-y(3)) ,-sin(-y(3)) ; sin(-y(3)) ,cos(-y(3))]*[eX;eY];
Error=[e(1) e(2) y(3) y(6) y(7)]';
u=-K*Error;
end