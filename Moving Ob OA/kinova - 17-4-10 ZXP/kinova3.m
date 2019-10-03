%ZXP ���ڼ����е�۸�����/�ؽڵ�λ�� j1px(,1)-j7pz(,1)
%ZXP ĩ��ִ�������ٶ� dpos_(,1)
%ZXP ִ�����������켣����� erpos_(),erdpos_()
%ZXP ��������VisualStudio���ƻ�е�۵� mvn_Datarun

clear all;
format long;
global d1 d2 lenK T mu r alpha q0;
load SYSdata;
load SNDdata;

Time=0;                             %Time����
t_last=0;
t_time=0;
[m,n]=size(qAll);
q_Time=zeros(m,1);

%q0
[Pox,Poy,Poz,PoTr]=positionN(q0,lenK);  %ZXP initial position
IntPos = [0, 0, 1, 0]*PoTr;    %ZXP inital normal vector of the end-effector
ix=Pox(7);iy=Poy(7);iz=Poz(7);

disp('The initial cartesain coordinate is');
ix
iy
iz

for jj=1:length(t),
   qjj=qAll(jj,:)';%simulated end effector position ZXP qAllΪ6���ؽڿ��ƽ�
   [Ppx,Ppy,Ppz,PpTr]=positionN(qjj,lenK);
   [xob_tjj(jj),yob_tjj(jj),zob_tjj(jj)] = Obstacle_t(t(jj));
   %-----------------
   j1px(jj,1)=Ppx(1);
   j2px(jj,1)=Ppx(2);
   j3px(jj,1)=Ppx(3);
   j4px(jj,1)=Ppx(4);
   j5px(jj,1)=Ppx(5);
   j6px(jj,1)=Ppx(6);
   j7px(jj,1)=Ppx(7);
   %-----------------
   j1py(jj,1)=Ppy(1);
   j2py(jj,1)=Ppy(2);
   j3py(jj,1)=Ppy(3);
   j4py(jj,1)=Ppy(4);
   j5py(jj,1)=Ppy(5);
   j6py(jj,1)=Ppy(6);
   j7py(jj,1)=Ppy(7);
   %-----------------
   j1pz(jj,1)=Ppz(1);
   j2pz(jj,1)=Ppz(2);
   j3pz(jj,1)=Ppz(3);
   j4pz(jj,1)=Ppz(4);
   j5pz(jj,1)=Ppz(5);
   j6pz(jj,1)=Ppz(6);
   j7pz(jj,1)=Ppz(7);
   %-----------------
   [J,JJ,JJJ]=JacobN(qjj,lenK);       %ZXP ��е�� �ſ˱�
   %check the virtual distance
   dqjj=dqAll(jj,:)';
   [JN,distob,rightB,effective_oan(jj)]=JObstacleN(qjj,dqjj,t(jj));  %ZXP ��е�۱��� �ſ˱�
   minD(jj)=min(distob);    %ZXP ��С�ϰ���-�ؽھ���
   [mind min_d_t]=min(minD);
   lessthan0(jj,:)=myabs(JN*dqjj)';  %ZXP Jo*thetaDot (<= bo)
   %-----------------
   dpos=J*dqjj;
   dposx(jj,1)=dpos(1);
   dposy(jj,1)=dpos(2);
   dposz(jj,1)=dpos(3);
   %ZXP calculate 'z' axis error
   zpos = PpTr*(IntPos');
   zpos = zpos(1:3);
   cosz(jj,:) = [0, 0, 1]*zpos;
   EndNor(jj,:) = 0.05*zpos';    %ZXP EndNor denotes the normal vector of the end-effector(in earth coordination), see my paper
   
t_time=t(jj)-Time*T;	%ÿ�������ڵ������������������ʱ�䣬�Ըñ�����ʼ����������ڱ������е���һ��ʱ��
% if t_time>T;
%    q_Time(jj,1)=1;
%    Time=Time+1;
%    t_time=t_time-T; %ZXP ������ =>t_time=t(jj)-Time*T;
% end
 

%    %--Desired rt and ddrt--  
%    phi_sin=2*pi*sin(0.5*pi*t_time/T);
%    phi=phi_sin*sin(0.5*pi*t_time/T);
%    phiDot=phi_sin*pi*cos(0.5*pi*t_time/T)/T;
%    rx(jj,1)=r*cos(phi)+ix-r;
%    ry(jj,1)=r*cos(alpha)*sin(phi)+iy;
%    rz(jj,1)=r*sin(alpha)*sin(phi)+iz;
%    rxdot(jj,1)=-r*sin(phi)*phiDot;
%    rydot(jj,1)=r*cos(phi)*cos(alpha)*phiDot;
%    rzdot(jj,1)=r*cos(phi)*sin(alpha)*phiDot;
   
if t_time<=(T)
    rx(jj,1) = r*cos(2*pi*t_time/T)+ix-r;
    ry(jj,1) = r*sin(2*pi*t_time/T)+iy;
    rz(jj,1) = iz;
    rxdot(jj,1) = -2*pi/T*r*sin(2*pi*t_time/T);
    rydot(jj,1) = 2*pi/T*r*cos(2*pi*t_time/T);
    rzdot(jj,1) = 0;
else   %ZXP ���
    rx(jj,1) = r*cos(2*pi*(T)/T)+ix-r;
    ry(jj,1) = r*sin(2*pi*(T)/T)+iy;
    rz(jj,1) = 0.05*(t_time-T)+iz;
    rxdot(jj,1) = 0;
    rydot(jj,1) = 0;
    rzdot(jj,1) = 0.05;
end

%    %--Desired rt and ddrt--
%    radiusXY0=0.12;
%    angleXY0=3*pi/2;
%    phi_sin=2*pi*sin(0.5*pi*t(jj)/T);
%    phi=phi_sin*sin(0.5*pi*t(jj)/T);
%    phiDot=phi_sin*pi*cos(0.5*pi*t(jj)/T)/T;
%    rxdot(jj,1)=-radiusXY0*sin(angleXY0+phi)*phiDot;
%    rydot(jj,1)=0.5*radiusXY0*cos(angleXY0+phi)*phiDot;rzdot=0+iz;
%    rzdot(jj,1)=0;
%    rx(jj,1)=radiusXY0*cos(angleXY0+phi)+ix-radiusXY0*cos(angleXY0); 
%    ry(jj,1)=0.5*radiusXY0*sin(angleXY0+phi)+iy-0.5*radiusXY0*sin(angleXY0); 
%    rz(jj,1)=0+iz;


end

%--Errors-
%ZXP endeffecter position error
erposx=rx-j7px;
erposy=ry-j7py;
erposz=rz-j7pz;
erRMSE = (erposx.^2+erposy.^2+erposz.^2).^(1/2);
%ZXP endeffecter postition velocity error
erdposx=rxdot-dposx;
erdposy=rydot-dposy;
erdposz=rzdot-dposz;
%ZXP 'z' axis calculation
zerror = (acos(cosz)/pi)*180;


%ZXP �ظ��˶�error
qError = qAll(m,1:6)-q0';
qError

save SRDdata1 t qAll dqAll ddqAll uAll rx ry rz dposx dposy dposz minD min_d_t lessthan0 effective_oan;
save SRDdata2 erposx erposy erposz erRMSE qError zerror EndNor erdposx erdposy erdposz;
save SRDdata3 j1px j2px j3px j4px j5px j6px j7px j1py j2py j3py j4py j5py ...
     j6py j7py j1pz j2pz j3pz j4pz j5pz j6pz j7pz xob_tjj yob_tjj zob_tjj;

%ZXP ����VisualStudio���ƻ�е��
%����Ϊ����ɸѡ�����
%�ȼ������ɸѡ
[m,n]=size(qAll);
step_long=(3e-3)^2;
StepTime = 0.05;
jjj=1;
k=1;
q_step(1,:)=qAll(1,:);
for iii=2:m
      deta=(j7px(iii)-j7px(k))*(j7px(iii)-j7px(k))+(j7py(iii)-j7py(k))*(j7py(iii)-j7py(k))+(j7pz(iii)-j7pz(k))*(j7pz(iii)-j7pz(k));
    if deta>step_long||q_Time(iii)==1
    %if abs(t(iii)-t(k)-StepTime)<0.005
        jjj=jjj+1;
        k=iii;
        q_step(jjj,:)=qAll(iii,:);
        t_step(jjj)=t(iii);
        q_deta(jjj,1)=deta;
        q_finish(jjj,1)=q_Time(iii);
        deta
    end
end

q_deta(1,1)=step_long;
%�������������Jaco��ģ��ʵ�ʵ�Jaco�г�ʼ0��ƫ���Լ���ת����ĵĲ��죬���������ȶ�ƫ�Ǻ���ת��������������ٱ�������
q_tran=[0*ones(jjj,1),(1/2*pi)*ones(jjj,1),(-1/2*pi)*ones(jjj,1),0*ones(jjj,1),(pi)*ones(jjj,1),(-10/36)*2*pi*ones(jjj,1),0*ones(jjj,1),0*ones(jjj,1)];
q_Real=[-q_step(:,1),q_step(:,2),q_step(:,3),q_step(:,4),q_step(:,5),q_step(:,6), q_deta, q_finish];
q_Real=q_Real+q_tran;

save mvn_Datarun q_Real;
disp('3 finished');
kinova4;