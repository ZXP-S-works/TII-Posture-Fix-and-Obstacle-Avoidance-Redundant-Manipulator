function [J, JJ,JJJ]=JacobN(q,len)
%ZXP J denotes jacobian matrix for position
%ZXP JJ denotes jacobian matrix for endeffector's z axis' posture

D = [0.2755, 0.4100, 0.2073, 0.07433, 0.07433, 0.13, 0.0098]; %D1~D6;E2 ZXP not exact
pen_long=0.0;                                 %笔长
aaa = pi*(60/360);
ca = cos(aaa);
sa = sin(aaa);
c2a = cos(2*aaa);
s2a = sin(2*aaa);
d4b = D(3)+sa/s2a*D(4);
d5b = sa/s2a*D(4)+sa/s2a*D(5);
d6b = sa/s2a*D(5)+D(6);
alpha_i = [pi/2, pi, pi/2, 2*aaa, 2*aaa, pi];%kinova本身自己各关节之间的夹角
a_i = [0, D(2), 0, 0, 0, 0];
d_i = [D(1), 0, -D(7), -d4b, -d5b, -d6b];
	mys=sin(q);
    s1=mys(1);s2=mys(2);s3=mys(3);s4=mys(4);s5=mys(5);s6=mys(6);
    myc=cos(q);
    c1=myc(1);c2=myc(2);c3=myc(3);c4=myc(4);c5=myc(5);c6=myc(6);
    
    alpha_4 = alpha_i(4);alpha_5 = alpha_i(5);
    d_1 = d_i(1);d_2 = d_i(2);d_3 = d_i(3);d_4 = d_i(4);
    d_5 = d_i(5);d_6 = d_i(6);
%     sp = sin(phi);
%     cp = cos(phi);
    sa4 = sin(alpha_4);ca4 = cos(alpha_4);
    sa5 = sin(alpha_5);ca5 = cos(alpha_5);
    
%以下的AW1和AW2实际上是由于机械臂的安装或建模问题最终引起了画图的一点小小误差，
%通过基点的一个旋转矩阵偏移一个小角度让笔尖画出了的轨迹在平面内
    sp1=sin(0.02);cp1=cos(0.02);
    sp2=sin(0.05);cp2=cos(0.05);
    
    AW1 = [1, 0, 0, 0;...
    0, cp1, sp1, 0;...
    0, -sp1, cp1, 0;...
    0, 0, 0, 1];
    
    AW2 = [cp2, 0, -sp2, 0;...
    0, 1, 0, 0;...
    sp2, 0, cp2, 0;...
    0, 0, 0, 1];

    AW1=AW1*AW2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
AW1=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
%以下为Jaco机械臂旋转矩阵
    A1 = [c1, 0, s1, 0;...
    s1, 0, -c1, 0;...
    0, 1, 0, len(1);...
    0, 0, 0, 1];
    DA1 = [-s1, 0, c1, 0;...
    c1, 0, s1, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA1 = [-c1, 0, -s1, 0;...
    -s1, 0, c1, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    
    A2 = [c2, s2, 0, len(2)*c2;...
    s2, -c2, 0, len(2)*s2;...
    0, 0, -1, 0;...
    0, 0, 0, 1];
    DA2 = [-s2, c2, 0, len(2)*-s2;...
    c2, s2, 0, len(2)*c2;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA2 = [-c2, -s2, 0, len(2)*-c2;...
    -s2, c2, 0, len(2)*-s2;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    
    A3 = [c3, 0, s3, 0;...
    s3, 0, -c3, 0;...
    0, 1, 0, len(3);...
    0, 0, 0, 1];
    DA3 = [-s3, 0, c3, 0;...
    c3, 0, s3, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA3 = [-c3, 0, -s3, 0;...
    -s3, 0, c3, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0]; 
    
    A4 = [c4, -ca4*s4, sa4*s4, 0;...
    s4, ca4*c4, -sa4*c4, 0;...
    0, sa4, ca4, len(4);...
    0, 0, 0, 1];
    DA4 = [-s4, -ca4*c4, sa4*c4, 0;...
    c4, ca4*-s4, sa4*s4, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA4 = [-c4, ca4*s4, sa4*-s4, 0;...
    -s4, ca4*-c4, sa4*c4, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    
    A5 = [c5, -ca5*s5, sa5*s5, 0;...
    s5, ca5*c5, -sa5*c5, 0;...
    0, sa5, ca5, len(5);...
    0, 0, 0, 1];
    DA5 = [-s5, -ca5*c5, sa5*c5, 0;...
    c5, ca5*-s5, sa5*s5, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA5 = [-c5, ca5*s5, sa5*-s5, 0;...
    -s5, ca5*-c5, sa5*c5, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];

    A6 = [c6, s6, 0, 0;...
    s6, -c6, 0, 0;...
    0, 0, -1, len(6);...
    0, 0, 0, 1];
    DA6 = [-s6, c6, 0, 0;...
    c6, s6, 0, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];
    DDA6 = [-c6, -s6, 0, 0;...
    -s6, c6, 0, 0;...
    0, 0, 0, 0;...
    0, 0, 0, 0];

q9=0;c9=cos(q9);s9=sin(q9);
A7=[c9,-s9,0,0;s9,c9,0,0;0,0,1,0;0,0,0,1];
    
J1=DA1*A2*A3*A4*A5*A6;
J2=A1*DA2*A3*A4*A5*A6;
J3=A1*A2*DA3*A4*A5*A6;
J4=A1*A2*A3*DA4*A5*A6;
J5=A1*A2*A3*A4*DA5*A6;
J6=A1*A2*A3*A4*A5*DA6;


% %ZXP DH model for Jaco2
% %ZXP code by ZXP in 2017.4.12
% 
% %ZXP DH parameters (refer 'DH Parameters - Kinova - 1.1.6.pdf')
% 
% D= [0.2755, 0.4100, 0.2073, 0.0743, 0.0743, 0.1687];    %ZXP D1~D6
% e2 = 0.0098;
% 
% aa = ((11*pi)/72);
% ca = cos(aa);
% sa = sin(aa);
% c2a = cos(2*aa);
% s2a = sin(2*aa);
% d4b = D(3)+sa/s2a*D(4);
% d5b = sa/s2a*D(4)+sa/s2a*D(5);
% d6b = sa/s2a*D(5)+D(6);
% 
% alpha = [pi/2, pi, pi/2, 2*aa, 2*aa, pi];
% a = [0, D(2), 0, 0, 0, 0];
% di = [D(1), 0, -e2, -d4b, -d5b, -d6b];
% Sa = sin(alpha);
% Ca = cos(alpha);
% Sq = sin(q);
% Cq = cos(q);
% 
% %ZXP transform matrix
% 
% A1 = [ Cq(1), 0,   Sq(1),     0;
%        Sq(1), 0,  -Cq(1),     0;
%        0,     1,       0, di(1);
%        0,     0,       0,     1 ];


%ZXP J denotes jacobian matrix for position
jb=[0;0;0;1];
K(:,1)=J1*jb;
K(:,2)=J2*jb;
K(:,3)=J3*jb;
K(:,4)=J4*jb;
K(:,5)=J5*jb;
K(:,6)=J6*jb;
J = K(1:3,:);
%ZXP JJ denotes jacobian matrix for endeffector's 'z' axis' posture, two
%ZXP two parameter is enough to represent 'z' axis' posture.
jb=[0,0,1,0];   %ZXP note the order
K(:,1)=(jb*J1)';
K(:,2)=(jb*J2)';
K(:,3)=(jb*J3)';
K(:,4)=(jb*J4)';
K(:,5)=(jb*J5)';
K(:,6)=(jb*J6)';
%JJ = K(1:3,:);
JJ = K(1:2,:);	%ZXP two parameter is enough to represent
JJJ = K(1:3,:); %ZXP for calculation