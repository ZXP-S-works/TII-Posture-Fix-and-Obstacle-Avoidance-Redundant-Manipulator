function doty=kinovaNet(t,y)
%ZXP y(1:6)=>q()=>qAll
%ZXP y(7:12)=>qDot()
%ZXP y(7:3*)=>u()

global d1 d2 lenK T mu m n r alpha xob yob zob newB oan;


for i=1:6,
   q(i,1)=y(i);
end
for i=1:6,
   u(i,1)=y(i+6);
   qDot(i,1)=y(i+6);
end
for i=7:(6+m+n+3*oan),
   u(i,1)=y(i+6);
end

if t<=(T)
%     phi_sin=2*pi*sin(0.5*pi*t/T);
%     phi=phi_sin*sin(0.5*pi*t/T);
%     phiDot=phi_sin*pi*cos(0.5*pi*t/T)/T;
%     xdot=-r*sin(phi)*phiDot;
%     ydot=r*cos(phi)*cos(alpha)*phiDot;
%     zdot=r*cos(phi)*sin(alpha)*phiDot;  %ZXP ������ 

%    x = r*cos(2*pi*t/T);
%    y = r*sin(2*pi*t/T);
    xdot = -2*pi/T*r*sin(2*pi*t/T);
    ydot = 2*pi/T*r*cos(2*pi*t/T);
    zdot = 0;
else   %ZXP ���
    xdot = 0;
    ydot = 0;
    zdot = 0.05;
end
    
% radiusXY0=0.12;
% angleXY0=3*pi/2;
% phi_sin=2*pi*sin(0.5*pi*t/T);
% phi=phi_sin*sin(0.5*pi*t/T);
% phiDot=phi_sin*pi*cos(0.5*pi*t/T)/T;
% xdot=-radiusXY0*sin(angleXY0+phi)*phiDot;
% ydot=0.5*radiusXY0*cos(angleXY0+phi)*phiDot;
% zdot=0;

% rx=radiusXY0*cos(angleXY0+phi)+ix-radiusXY0*cos(angleXY0); 
% ry=0.5*radiusXY0*sin(angleXY0+phi)+iy-0.5*radiusXY0*sin(angleXY0); 
% rz=0+iz;
 


dr=[xdot;ydot;zdot];

[J,JJ,JJJ] = JacobN(q,lenK);   %ZXP J,JJΪ3x6���� J=>A JJ=>AA
[JN,distob,sdjt,effective_oan,dis]=JObstacleN(q,qDot,t);
% ZXP JN=>Jo JNΪ3*oan x6���� ��18x6
% ZXP distobf С��d2�� critical point �� obstacle�ľ���
% ZXP rightB=>bo=>sdjt
% ZXP effective_oan ���ϰ���������d2��������
% ZXP distob critical point �� obstacle�ľ���
% effective_oan

if(mod(t,1) < 0.001)
    t
end

%size(sdjt)
% JJJ = 0*JJJ; JJ  = 0*JJ;    %ZXP for no 'z' axis fixation
JN = 0*JN;    %ZXP for no obstacle avoidance
WWW=zeros(6,6);
HH=[WWW -J' -JJJ' JN'; J zeros(m,m+n+3*oan); JJJ zeros(n,m+n+3*oan); -JN zeros(3*oan,m+n+3*oan)]; %ZXP HH=>M?
%ZXP For obstacle avoidance.
%ZXP 
%       [ Q     -[A ]T	-[AA]T	CT	]
%   M = [ [A ]	0       0       0   ]
%       [ [AA]  0       0       0   ]
%       [ -C 	0       0       0   ]
dd = zeros(1,n); %ZXP dd denotes desire endeffector's z axis posture
pp=[zeros(6,1); -dr; -dd'; sdjt]; %ZXP PP=>q

global qP qM qDp qDm;
beta=0.25;%or 1 or 2
qCp=beta*(qP-q);
qCm=beta*(qM-q);
for i=1:6,%���������ƣ��ؽ�+�ٶ�
    if (qCp(i)>qDp(i)) qCp(i)=qDp(i);
    end;
    if (qCm(i)<qDm(i)) qCm(i)=qDm(i);
    end;
end;
ym=[qCm; -inf*ones(m+n,1); zeros(3*oan,1)];   %ZXP u-
yp=[qCp; inf*ones(m+n,1); inf*ones(3*oan,1)]; %ZXP u+
ImH=eye(size(HH))-HH;
IpHt=eye(size(HH))+HH'; %ZXP ������
uDot=mu*IpHt*(gfunction(ym,ImH*u-pp,yp)-u); %ZXP LVI-PDNN
doty=[qDot;uDot];   %ZXP y(1:6)Ϊ��qDot���֣�y(7:33)ΪuDot�����磨����