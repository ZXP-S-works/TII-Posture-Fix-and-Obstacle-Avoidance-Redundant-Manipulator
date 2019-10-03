load QALL1;
load QALL2;
load QALL3;
load QALL4;

%ZXP UOA_UPF_2PF_3PF
figure;
plot(t1,qAll1(:,2));hold on;
plot(t2,qAll2(:,2));hold on;
plot(t4,qAll4(:,2));hold on;    %ZXP ×¢ÒâË³Ðò
plot(t3,qAll3(:,2));hold on;
plot([0,10],[qP(2),qP(2)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(2),qM(2)],'r--', 'LineWidth',1.5);
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UOA-UPM T2','AOA-UPM T2','2PF T2','3PF T2','max2','min2');

figure;
plot(t1,qAll1(:,3));hold on;
plot(t2,qAll2(:,3));hold on;
plot(t4,qAll4(:,3));hold on;
plot(t3,qAll3(:,3));hold on;
plot([0,10],[qP(3),qP(3)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(3),qM(3)],'r--', 'LineWidth',1.5);
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UOA-UPM T3','AOA-UPM T3','2PF T3','3PF T3','max3','min3');

%ZXP UOA_AOA
figure;
plot(t1,qAll1(:,2));hold on;
plot(t2,qAll2(:,2));hold on;
plot([0,10],[qP(2),qP(2)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(2),qM(2)],'r--', 'LineWidth',1.5);
title('UOA_AOA theta2 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UOA T2','AOA T2','max','min');

figure;
plot(t1,qAll1(:,3));hold on;
plot(t2,qAll2(:,3));hold on;
plot([0,10],[qP(3),qP(3)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(3),qM(3)],'r--', 'LineWidth',1.5);
title('UOA_AOA theta3 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UOA T3','AOA T3','max','min');

%ZXP UPF_2PF_3PF
figure;
plot(t2,qAll2(:,2));hold on;
plot(t3,qAll3(:,2));hold on;
plot(t4,qAll4(:,2));hold on;
plot([0,10],[qP(2),qP(2)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(2),qM(2)],'r--', 'LineWidth',1.5);
title('UPF_2PF_3PF theta2 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UPF T2','2PF T2','3PF T2','max','min');

figure;
plot(t2,qAll2(:,3));hold on;
plot(t3,qAll3(:,3));hold on;
plot(t4,qAll4(:,3));hold on;
plot([0,10],[qP(3),qP(3)],'r:', 'LineWidth',2.5);
plot([0,10],[qM(3),qM(3)],'r--', 'LineWidth',1.5);
title('UPF_2PF_3PF theta3 with physical constrains');
xlabel('t (Second)');
ylabel('angle (Rad)');
legend('UPF T3','2PF T3','3PF T3','max','min');