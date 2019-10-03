%ZXP ���ڴ�kinovaNet���ؼ��㡢�洢 t qAll dqAll ddqAll uAll��
clear;
format long;
load SYSdata;
load INITdata;

jj=1;
for ii=1:1:length(t),
    tn(jj,1)=t(ii,1);
    yn(jj,:)=y(ii,:);
    jj=jj+1;
end
clear t y;
t=tn;y=yn;
clear tn yn;    %ZXP ���ã�����

for ii=1:length(t),    
    doty=kinovaNet(t(ii,1),y(ii,:))';
    dqAll(ii,:)=doty(1,1:6);
    ddqAll(ii,:)=doty(1,7:12);  %ZXP ������
end

sizeddqAll=size(ddqAll)

figure;
plot(t,dqAll);title('t - dqAll');
xlabel('t (Second)');ylabel('dqAll');

figure;
plot(t,ddqAll);title('t - ddqAll');
xlabel('t (Second)');ylabel('ddqAll');

qAll=y(:,1:6);
sizeqAll=size(qAll)

qAll4=y(:,1:6);
sizeqAll=size(qAll)
t4 = t;

dqAll=y(:,7:(6+6));
sizedqAll=size(dqAll)

uAll=y(:,13:(6+6+m+n+oan*3));
sizeuAll=size(uAll)

save SNDdata t qAll dqAll ddqAll uAll;
save QALL4 t4 qAll4;
disp('2 finished');
kinova3;