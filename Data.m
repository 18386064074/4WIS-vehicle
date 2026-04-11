
clear all
clc

%Wheel coordinates
x1=1.25;y1=0.75;
x2=-1.25;y2=0.75;
x3=-1.25;y3=-0.75;
x4=1.25;y4=-0.75;
Kd=1;
Xicr=0,Yicr=10;
s_ang1=0;  
s_ang2=0;
s_ang3=0;
s_ang4=0;


%Main vehicle parameters
g=9.8;
la=x1;
lb=x1;
Iz=1600;
Iw=0.5;
m=1000;
B=(y1-y4)/2;            
Fz=la/(la+lb)*m*g/2;
vx=25/3.6;
% k=(m*u^2*lb*Cr+2*(la+lb)*la*Cr*Cf)/(m*u^2*Cf*la-2*(la+lb)*lb*Cf*Cr) 


%Steering motor parameters
JL=0.08;
Jm=6.96e-6;
Kb=0.0525;
Km=0.053;
Kf=1.4e-3;
L=6.2e-4;
R=2.07;
n=1/200;
Pulse=180;              %Motor disturbance
delay_Pulse_time=1.5;   %Pulse delay time
time=5;               
Ti=50;                  
r=0.32;                 
Fi=Ti/r;                
  
%PID parameters
pd=100;
id=0.01;
dd=0.01;

pk=300;
ik=1;
dk=2;

ph=300;
ih=1;
dh=2;

%Aerodynamic drag coefficient
As=2.15;
Cd=0.28;

%Tire formula parameters
u=0.85;


%模式选择
modeli=1;            
derta1=0.5;
clockwise=1;           



x1_fc=0.01;x1_jz=0;
x2_fc=0.05;x2_jz=0.01;
x3_fc=0.1;x3_jz=0.05;
x4_fc=0.5;x4_jz=0.08;
