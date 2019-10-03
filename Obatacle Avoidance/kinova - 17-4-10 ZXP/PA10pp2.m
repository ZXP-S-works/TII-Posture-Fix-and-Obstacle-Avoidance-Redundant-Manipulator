clear;
format long;
load SYSdata;
load INITdata;

interval=0.1;

jj=0;
epsilon=0;
for ii=1:length(t),
    if(t(ii,1)>=epsilon)
        jj=jj+1;
        tn(jj,1)=t(ii,1);
        yn(jj,:)=y(ii,:);
        epsilon=jj*interval;
    elseif(ii==length(t))
        jj=jj+1;
        tn(jj,1)=t(ii,1);
        yn(jj,:)=y(ii,:);
        epsilon=jj*interval;
    end
end
clear t y;
t=tn;y=yn;
clear tn yn;

for ii=1:length(t),    
    doty=PA10net(t(ii,1),y(ii,:))';
    dqAll(ii,:)=doty(1,1:8);
    ddqAll(ii,:)=doty(1,9:16);
end
size(ddqAll)
figure;plot(t,dqAll)
figure;plot(t,ddqAll)

qAll=y(:,1:8);
size(qAll)

dqAll=y(:,9:(8+8));
size(dqAll)

uAll=y(:,17:(8+8+3+oan*3));
size(uAll)

save SNDdata t qAll dqAll ddqAll uAll;