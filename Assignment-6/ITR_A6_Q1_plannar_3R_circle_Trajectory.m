%%
clear;close all;clc
%% Circle Trajectory
dt=1/100;
t=-6+dt:dt:6+dt;
scale = 1.5
x = scale.*cos(t);
y = scale.*sin(t);
dx=diff(x)./dt;
dy=diff(y)./dt;
v=[dx;dy];
KX=[1 0;0 0];

%% Infinity Trajectory
% dt=1/100;
% t=0:dt:12+dt;
% scale = 2./(3 - cos(2*t));
% x = scale.*cos(t);
% y = scale.*sin(2*t)./ 2;
% x=x*3;
% y=y*3;
% dx=diff(x)./dt;
% dy=diff(y)./dt;
% v=[dx;dy];
% KX=[1 0;0 0];

%%
ath=0;
K=[cosd(ath) 0;sind(ath) 0];
kxx=K(1,1);
kxy=K(1,2);
kyx=K(2,1);
kyy=K(2,2);
%% Feedforward Velocity Control (FVC)
q=[0;0;0];
for k=1:1:length(x)-1
    th1=q(1,k);th2=q(2,k);th3=q(3,k);
    J = jaco_3(th1,th2,th3);
    q(:,k+1)=q(:,k) + 1*pinv(J)*(v(:,k))*dt ;
end
animate_3r(q,K,x,y)

%% FVC with initial offset (disturbance)
q=[pi/32;0;0];
for k=1:1:length(x)-1
    th1=q(1,k);th2=q(2,k);th3=q(3,k);
    J = jaco_3(th1,th2,th3);
    q(:,k+1)=q(:,k) + 1*pinv(J)*(v(:,k))*dt ;
end
%animate_3r(q,K,x,y)

%% Feedback control
q=[pi/32;1;0];
for k=1:1:length(x)-1
    th1=q(1,k);th2=q(2,k);th3=q(3,k);
    J = jaco_3(th1,th2,th3);
    [xo,yo] = fwd_kin3(q(:,k));
    E=[x(k);y(k)]-[xo;yo];
    q(:,k+1)=q(:,k) + 1*pinv(J)*(v(:,k) + 1E2*E)*dt ;
end
animate_3r(q,K,x,y)

%% Manipulability Ellipsoid
function [Xo,Yo,Xof,Yof]=ellipsoid(J,x0,y0)
[evt,evl]=eig(J*J');
if(evl(2,2)>evl(1,1))
    a=sqrt(evl(2,2));
    b=sqrt(evl(1,1));
    ax=1;
    ay=0;
    bx=evt(1,2);
    by=evt(2,2);
    th=atan2( ax*by-ay*bx, ax*bx+ay*by );
end
if(evl(2,2)<evl(1,1))
    a=sqrt(evl(1,1));
    b=sqrt(evl(2,2));
    ax=1;
    ay=0;
    bx=evt(1,2);
    by=evt(2,2);
    th=atan2( ax*by-ay*bx, ax*bx+ay*by );
end
thf= th + pi/2;
te=-pi:0.01:pi;
x56=a*cos(te);
y56=b*sin(te);
R5=[cos(th) -sin(th) ; sin(th) cos(th)];
P5= R5*[x56;y56];
Xo= P5(1,:)+x0; 
Yo= P5(2,:)+y0; 
R5=[ cos(thf) -sin(thf) ; sin(thf) cos(thf)];
P5= R5*[x56;y56];
Xof= P5(1,:)+x0; 
Yof= P5(2,:)+y0; 
end

%% Jacobian - 2R, 3R
function J = jaco_2(th1,th2)
l1=1;l2=1;
J=[-l1*sin(th1)-l2*sin(th2+th1),-l2*sin(th1+th2);l1*cos(th1)+l2*cos(th1+th2),l2*cos(th1+th2)];
end
function J = jaco_3(th1,th2,th3)
l1=1;l2=1;l3=1;
J=[-l1*sin(th1)-l2*sin(th2+th1)-l3*sin(th3+th2+th1),-l2*sin(th2+th1)-l3*sin(th3+th2+th1),-l3*sin(th1+th2+th3);l1*cos(th1)+l2*cos(th2+th1)+l3*cos(th3+th2+th1),l2*cos(th2+th1)+l3*cos(th3+th2+th1),l3*cos(th1+th2+th3)];
end

%% FK - 2R, 3R
function [x,y] = fwd_kin2(q)
l1=1;l2=1;
x=l1*cos(q(1,:)) + l2*cos(q(1,:)+q(2,:));
y=l1*sin(q(1,:)) + l2*sin(q(1,:)+q(2,:));
end
function [x,y] = fwd_kin3(q)
l1=1;l2=1;l3=1;
x=l1*cos(q(1,:)) + l2*cos(q(1,:)+q(2,:)) + l3*cos(q(1,:)+q(2,:)+q(3,:));
y=l1*sin(q(1,:)) + l2*sin(q(1,:)+q(2,:)) + l3*sin(q(1,:)+q(2,:)+q(3,:));
end

%% Animation
function []=animate_3r(q,K,xt,yt)
xi=q;
figure('WindowState','maximized')
%pause(7)
c=1;
for i=1:15:length(xi)
    theta1=xi(1,i);
    theta2=xi(2,i);
    theta3=xi(3,i);
    l1=1; %Input the l length
    l2=1; %Input the l length
    l3=1; %Input the l length
    % Homogeneus transformation matrix
    H01 = [cos(theta1) -sin(theta1) 0 l1*cos(theta1);sin(theta1) cos(theta1) 0 l1*sin(theta1);0 0 1 0;0 0 0 1]; %Frame 0 to 1 tranformation
    H12 = [cos(theta2) -sin(theta2) 0 l2*cos(theta2);sin(theta2) cos(theta2) 0 l2*sin(theta2);0 0 1 0;0 0 0 1]; %Frame 1 to 2 tranformation
    H23 = [cos(theta3) -sin(theta3) 0 l3*cos(theta3);sin(theta3) cos(theta3) 0 l3*sin(theta3);0 0 1 0;0 0 0 1]; %Frame 1 to 2 tranformation
    H02=H01*H12;      %Frame 0 to 2 tranformation
    H03=H01*H12*H23;

    P1=[H01(1,4) H01(2,4)];
    P2=[H02(1,4) H02(2,4)];
    P3=[H03(1,4) H03(2,4)];
    P3x(c,:)=P3(1,1);
    P3y(c,:)=P3(1,2);
    plot(xt,yt,'--y')
    hold on
    plot(P1(1),P1(2),'ok','LineWidth',1)

    plot(P3x,P3y,'k')
    plot(P2(1),P2(2),'ok','LineWidth',1)
    plot(P3(1),P3(2),'ok','LineWidth',1)

    J=[-l1*sin(theta1)-l2*sin(theta2+theta1)-l3*sin(theta3+theta2+theta1),-l2*sin(theta2+theta1)-l3*sin(theta3+theta2+theta1),-l3*sin(theta1+theta2+theta3);l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1),l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1),l3*cos(theta1+theta2+theta3)];
    [~,~,Xof,Yof]=ellipsoid(J/5,P3(1),P3(2));
    [Xofk,Yofk,~,~]=ellipsoid(K/2,P3(1),P3(2));
    plot(0,0,'ok','LineWidth',3)
%     plot(Xof,Yof,'r')
%     plot(Xofk,Yofk,'m')
    xlim([-3.5 3.5])
    ylim([-3.5 3.5])
    axis square; 
    grid minor
    plot([0 P1(1)], [0 P1(2)],'g','LineWidth',2)
    plot([P1(1) P2(1)], [P1(2) P2(2)],'b','LineWidth',2)
    plot([P2(1) P3(1)], [P2(2) P3(2)],'g','LineWidth',2)
    xlabel('X axis (m)','Interpreter','latex')
    ylabel('Y axis (m)','Interpreter','latex')
    set(gca,'FontSize',18)
    drawnow
    hold off
    c=c+1;
end
end
