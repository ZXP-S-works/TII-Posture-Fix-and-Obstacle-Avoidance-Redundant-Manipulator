function [A1,A2,A3,A4,A5,A6]=TransformN(q,len)
D = [0.2755, 0.4100, 0.2073, 0.07433, 0.07433, 0.13, 0.0098]; %D1~D6;E2
pen_long=0.0;                                 %笔长
aaa = 11.0*pi/72;
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
    
    %Jein6=[0;-pen_long;0;1];%末端为笔
    Jein6=[0;0;0;1];%末端为指尖
    J6in5=[0;0;-D(5)*sa/s2a;1];
    J5in4=[0;0;-D(4)*sa/s2a;1];  
    J4in3=[0;0;-D(3);1];
    J3in2=[0;0;0;1];
    J2in1=[0;0;0;1];    
    J1inc=[0;0;0;1];   
    
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
AW1=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
    
    T01 = [c1, 0, s1, 0;...
    s1, 0, -c1, 0;...
    0, 1, 0, len(1);...
    0, 0, 0, 1];
A1=T01*AW1;

	T12 = [c2, s2, 0, len(2)*c2;...
    s2, -c2, 0, len(2)*s2;...
    0, 0, -1, 0;...
    0, 0, 0, 1];
A2=T12*AW1;

	T23 = [c3, 0, s3, 0;...
    s3, 0, -c3, 0;...
    0, 1, 0, len(3);...
    0, 0, 0, 1];
A3=T23*AW1;

	T34 = [c4, -ca4*s4, sa4*s4, 0;...
    s4, ca4*c4, -sa4*c4, 0;...
    0, sa4, ca4, len(4);...
    0, 0, 0, 1];
A4=T34*AW1;

	T45 = [c5, -ca5*s5, sa5*s5, 0;...
    s5, ca5*c5, -sa5*c5, 0;...
    0, sa5, ca5, len(5);...
    0, 0, 0, 1];
A5=T45*AW1;

	T56 = [c6, s6, 0, 0;...
    s6, -c6, 0, 0;...
    0, 0, -1, len(6);...
    0, 0, 0, 1];
A6=T56*AW1;

q9=0;c9=cos(q9);s9=sin(q9);
A7=[c9,-s9,0,0;s9,c9,0,0;0,0,1,0;0,0,0,1];

    Peeff=AW1*A1*A2*A3*A4*A5*A6*A7*Jein6;
    %j7p=AW1*A1*A2*A3*A4*A5*A6*J6in5;
    j6p=AW1*A1*A2*A3*A4*A5*J5in4;
    j5p=AW1*A1*A2*A3*A4*J4in3;
    j4p=AW1*A1*A2*A3*J3in2;
    j3p=AW1*A1*A2*J2in1;
    j2p=AW1*A1*J1inc;
    j1p=AW1*J1inc;
    
     x=[j1p(1);j2p(1);j3p(1);j4p(1);j5p(1);j6p(1);Peeff(1)];
     y=[j1p(2);j2p(2);j3p(2);j4p(2);j5p(2);j6p(2);Peeff(2)];
     z=[j1p(3);j2p(3);j3p(3);j4p(3);j5p(3);j6p(3);Peeff(3)];

     T=AW1*A1*A2*A3*A4*A5*A6*A7;