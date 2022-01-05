function [A,B,LMF,NoSol,xx,yy]=getpath(map,r0,mm,m)

x=map(:,2);
y=map(:,1);
w=map(:,3);
R=[];
A1=[];
GoalY=y(length(x));
GoalX=x(length(x));
slope_r0=(m/6+1.5-1)/(m-1+eps);
coefs_r0=slope_r0*(mm-1)+1;
r0=coefs_r0*r0;
for i=1:100
    yy(i)=(GoalY/GoalX)*i/10;
    
end
for I=1:100
    for J=1:100
        R=[];
        a=[];
        for r=1:length(x)-1
            %R=[R w(r)/((J/10-y(r))^2+(I/10-x(r))^2)];
            % R=[R w(r)/((J/10-y(r))^2+(I/10-x(r))^2)];
            ro(r)=sqrt((J/10-y(r))^2+(I/10-x(r))^2);
            %r0=5;
            
            if ro(r)<r0
                R=[R w(r)*(1/ro(r)-1/r0)*1/(ro(r)^2)];
                %  R=[R 0.5*w(r)*((1/ro(r)-1/r0)^2)];
            else
                R=[R 0];
                
            end
        end
        
        sign_v=[1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1];
        rg=sqrt((J/10-yy(I))^2);
        RG = sqrt((J/10-GoalY-sign_v(mm))^2+(I/10-GoalX+sign_v(mm))^2);
        % RG = 0.5*((J/10-GoalY)^2+(I/10-GoalX)^2);
        
        slope=(2.6-1.4)/(m-1+eps);
        coefs=slope*(mm-1)+1.4;
        
        A1(I,J) = sum(R)+ exp(RG/coefs)+exp(rg/1.2/coefs);%+rg/m;%exp(RG/3)*RG+exp(RG/m)*rg;
        if (A1(I,J)>10000)
            A1(I,J)=10000;
        end
        
    end
end

i=1:100;
j=1:100;
A=A1;
LMF=1;
tic
while(LMF==1)
    % Searching the path
    a=1;
    b=1;
    time_error=toc;
    if time_error>4
        break
    end
    LMF=0;
    a2=0;
    b2=0;
    B=zeros(length(A));
    value=(abs(a-GoalX*10)+abs(b-GoalY*10));
    while value>0   %target error
        [a1,b1] = checking(A,a,b,100,100);
        if((a1==a)&(b1==b))    %Local min or target
            a=a1;
            b=b1;
            B(a,b)=w(length(w));
            LMF=0;
            break;
        else
            if(a1==a2) & (b1==b2)   %Local min exist
                LMF=1;
                A(a1,b1)=600;
                % B(a1,b1)=0;
                break;
            end
            a2=a;
            b2=b;
            a=a1;
            b=b1;
            B(a,b)=w(length(w));
        end
        value=(abs(a-GoalX*10)+abs(b-GoalY*10));
    end
    
    % if(LMF==1)
    %     display('Local Minimum exists')
    % end
end

xx=[];
yy=[];
for ii=1:100
    for jj=1:100
        if B(ii,jj)~=0
            xx=[xx ii];
            yy=[yy jj];
        end
    end
end

NoSol=0;
for ii=1:100
    for jj=1:100
        if (B(ii,jj)==w(length(w)))
            if (A1(ii,jj)==600)
                NoSol=1;
            end
        end
    end
end

return
