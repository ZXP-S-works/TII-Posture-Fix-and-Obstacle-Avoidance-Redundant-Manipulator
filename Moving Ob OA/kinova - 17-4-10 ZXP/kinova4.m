%ZXP Plot some important parameters

clear all;
format long;
global d1 d2 lenK T mu r alpha q0;
load SYSdata;

load SRDdata1;
load SRDdata2;
load SRDdata3;

%==plotting angles==
figure;
plot(t,minD,':k','LineWidth',2);hold on;
% axis([0 10 0.04 0.16]);
line([0,10],[d2,d2],'color','blue','LineWidth',2);
line([0,10],[d1,d1],'color','red','LineWidth',4);
title('Minimum OC Distance');
xlabel('t (Second)');
legend('d','d1','d2');

figure;
plot(t,erRMSE,'k');hold on;
plot(t,erposx,':m','LineWidth',2);hold on;
plot(t,erposy,'-.b');hold on;
plot(t,erposz,'--r');hold on;
title('End-Effector Position Error');
xlabel('t (Second)');
legend('RMSE','ex','ey','ez');

figure;
plot(t,zerror,'k','LineWidth',1);hold on;
%axis([0 10 -2.5 2.5]);
title('z axis error');
xlabel('t (Second)');
ylabel('Posture error (Degree)');
title('End-effector Posture Error');

figure;
plot(t,qAll(:,2));hold on;
plot([0,10],[qP(2),qP(2)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(2),qM(2)],'r--', 'LineWidth',1.5);
%axis([0 10 -2.5 2.5]);
title('Simulated theta2 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('The2','max','min');

max2 = max(qAll(:,2))
min2 = min(qAll(:,2))

figure;
plot(t,qAll(:,3));hold on;
plot([0,10],[qP(3),qP(3)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(3),qM(3)],'r--', 'LineWidth',1.5);
%axis([0 10 -2.5 2.5]);
title('Simulated theta3 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('The3','max','min');

max3 = max(qAll(:,3))
min3 = min(qAll(:,3))

figure;
plot(t,effective_oan,'o');hold on;
title('the number of effective obstacle-link pairs');
xlabel('t (Second)');

figure;
plot3(j7px,j7py,j7pz,':k','LineWidth',2);hold on;
plot3(rx,ry,rz,'r');hold on;
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
% axis([0.15 0.3 -0.5 -0.3]);
legend('End-effector path','Desired path');
title('Desired Path and End-effector Path');

[Ppx,Ppy,Ppz,PpTr]=positionN(qAll(1,:),lenK);
pos_initial=[Ppx Ppy Ppz]
disp('4 finished');
kinova5;