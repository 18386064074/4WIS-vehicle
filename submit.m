%%
% target steering angle
function [deta4,deta1,deta3,deta2] = fcn(Yicr,Xicr)

x=[1.25,-1.25,-1.25,1.25];
y=[0.75,0.75,-0.75,-0.75];

if Yicr>0
    u1=1
    deta1=atan2(u1*(x(1)-Xicr),-u1*(y(1)-Yicr));
    deta2=atan2(u1*(x(2)-Xicr),-u1*(y(2)-Yicr));
    deta3=atan2(u1*(x(3)-Xicr),-u1*(y(3)-Yicr));
    deta4=atan2(u1*(x(4)-Xicr),-u1*(y(4)-Yicr));
else
    u1=-1
    deta1=atan2(u1*(x(1)-Xicr),-u1*(y(1)-Yicr));
    deta2=atan2(u1*(x(2)-Xicr),-u1*(y(2)-Yicr));
    deta3=atan2(u1*(x(3)-Xicr),-u1*(y(3)-Yicr));
    deta4=atan2(u1*(x(4)-Xicr),-u1*(y(4)-Yicr));
end

%%
% longitudinal velocity of the vehicle Vx
function dVx= fcn(Fx1,Fx2,Fx3,Fx4,m,w,Vy,vx,Cd,As)
dVx=(Fx1+Fx2+Fx3+Fx4-As*Cd*vx^2/21.15)/m+Vy*w;

%%
% lateral velocity of the vehicle Vy
function dVy= fcn(Fy1,Fy2,Fy3,Fy4,m,w,Vx)
dVy=(Fy2+Fy1+Fy3+Fy4)/m-Vx*w;

%%
% yaw rate Omega
function dW= fcn(Fx1,Fx2,Fx3,Fx4,Fy1,Fy2,Fy3,Fy4,Iz,la,lb,B)
dW=((Fy4+Fy1)*la-(Fy2+Fy3)*lb+(Fx4+Fx3)*B-(Fx1+Fx2)*B)/Iz;

%%
% one of the wheel's model
function f1 = tire(Fz,u,nameda1,alpha1,u1)
nameda1 = -nameda1;
cx = -4;
cy = -20;
L = (1-nameda1)*(1-0.015*u1*sqrt((cx^2)*(nameda1^2)+(cy^2)*((tan(alpha1))^2)))/(2*sqrt((cx^2)*(nameda1^2)+(cy^2)*((tan(alpha1))^2)));
if L<1
    fL = L*(2-L);
else
    fL = 1;
end
Fx0 = Fz*cx*nameda1*fL/(1-nameda1);
Fy0 = Fz*cy*tan(alpha1)*fL/(1-nameda1);
Fx = u*Fx0;
f1 = u*Fy0;

%%
% tire sideslip angle
function [a1,a2,a3,a4] = fcn(Vx,Vy,w,deta1,deta2,deta3,deta4,la,lb,B)
a1=atan((Vy+la*w)/(Vx-w*B))-deta1;
a2=atan((Vy-lb*w)/(Vx-w*B))-deta2;
a3=atan((Vy-la*w)/(Vx+w*B))-deta3;
a4=atan((Vy+la*w)/(Vx+w*B))-deta4;


%%
% energy loss by lateral slip of wheel
function [Ph1,Ph2,Ph3,Ph4] = fcn(Fy1,Fy2,Fy3,Fy4,Vx,a1,a2,a3,a4)
Ph1=-Fy1*Vx*a1;
Ph2=-Fy2*Vx*a2;
Ph3=-Fy3*Vx*a3;
Ph4=-Fy4*Vx*a4;

% energy loss by longitudinal slip of wheel
function [P1,P2,P3,P4] = fcn(nameda1,nameda2,nameda3,nameda4,u1,u2,u3,u4,Fx1,Fx2,Fx3,Fx4)
P1=Fx1*nameda1*u1;
P2=Fx2*nameda2*u2;
P3=Fx3*nameda3*u3;
P4=Fx4*nameda4*u4;

%%
% the center speed of wheel
function [u1,u2,u3,u4] = fcn(deta1,deta2,deta3,deta4,la,lb,Vx,Vy,w,B)
u1=(Vx-w*B)*cos(deta1)+(Vy+w*la)*sin(deta1);
u2=(Vx-w*B)*cos(deta2)+(Vy-w*lb)*sin(deta2);
u3=(Vx+w*B)*cos(deta3)+(Vy-w*lb)*sin(deta3);
u4=(Vx+w*B)*cos(deta4)+(Vy+w*la)*sin(deta4);

%%
% slip ratio of wheel
function [nameda1,nameda2,nameda3,nameda4] = fcn(u1,u2,u3,u4,Vx)
nameda1=(Vx-u1)/Vx;
nameda2=(Vx-u2)/Vx;
nameda3=(Vx-u3)/Vx;
nameda4=(Vx-u4)/Vx;




