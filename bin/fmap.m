function map=fmap(s,D,m,w)

% m1=s(1):(e(1)-s(1))/10:e(1);
% m2=s(2):(e(2)-s(2))/10:e(2);
r=D/2/100;
m1=[];
m2=[];
for x=(s(1)-r):0.01:(s(1)+r)
    for y=(s(2)-r):0.1:(s(2)+r)
        if sqrt((x-s(1)).^2 + (y-s(2)).^2)<=r
           m1=[m1 x]; 
                      m2=[m2 y]; 
        end
    end
end

%m1=linspace(s(1),e(1),abs((e(1)-s(1))*3));
%m2=linspace(s(2),e(2),abs((e(2)-s(2))*3));
%m3=max(length(m1),length(m2));
%m1=linspace(s(1),e(1),m3);
%m2=linspace(s(2),e(2),m3);
map=m;
for i1=1:length(m1)
    mm=horzcat(m1(i1).',m2(i1).',w);
    map=[map;mm ];
end