clear all
clc

syms Yicr Xicr 
    a=[-1,1/3,0.1,-3];
    b=[3,5/3,1,5];
    X=-5:1:5;
    for i=1:4
        Y=a(i)*X+b(i);
        plot(X,Y);
        hold on
        grid on
        xlim([-2,2])
    end
    legend('1','2','3','4')

    aicr=[-1/a(1),-1/a(2),-1/a(3),-1/a(4)];
    bicr=[(Yicr-aicr(1)*Xicr),(Yicr-aicr(2)*Xicr),(Yicr-aicr(3)*Xicr),(Yicr-aicr(4)*Xicr)];
    x=sym(zeros(1,4));
    y=sym(zeros(1,4));
    for i=1:4
        x(i)=-(bicr(i)-b(i))/(aicr(i)-a(i));
        y(i)=(aicr(i)*b(i)-a(i)*bicr(i))/(aicr(i)-a(i));
    end

    % z=x(3);
    % x(3)=x(4);
    % x(4)=z
    % z=y(3);
    % y(3)=y(4);
    % y(4)=z

    S2=1/2*(x(1)*y(2)-x(2)*y(1)-x(1)*y(3)+x(3)*y(1)+x(2)*y(4)-x(4)*y(2)-x(3)*y(4)+x(4)*y(3));
    dxicr=diff(S2,Xicr)
    dyicr=diff(S2,Yicr)
    f1=dyicr
    f2=dxicr
    [xicr,yicr]=solve(f1,f2,Xicr,Yicr);
