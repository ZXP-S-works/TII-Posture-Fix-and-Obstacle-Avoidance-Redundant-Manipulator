clear all;
format long;
global lenK xob yob zob T;
load SYSdata;

load SRDdata1;
load SRDdata2;
load SRDdata3;

figure;
plot3(xob,yob,zob,'or','Markersize',10,'linewidth',2);hold on;
plot3(j7px,j7py,j7pz,'r','linewidth',2);hold on;
i = 0;

for j=1:length(t)
    if abs(t(j)-i)<=(1e-1)
        linksX=[j1px(j);j2px(j);j3px(j);j4px(j);j5px(j);j6px(j);j7px(j)];
        linksY=[j1py(j);j2py(j);j3py(j);j4py(j);j5py(j);j6py(j);j7py(j)];
        linksZ=[j1pz(j);j2pz(j);j3pz(j);j4pz(j);j5pz(j);j6pz(j);j7pz(j)];
        plot3(linksX,linksY,linksZ,'k','linewidth',0.1);hold on;
        i = i+(T/15);
    end;
end;

j = min_d_t;
linksX=[j1px(j);j2px(j);j3px(j);j4px(j);j5px(j);j6px(j);j7px(j)];
linksY=[j1py(j);j2py(j);j3py(j);j4py(j);j5py(j);j6py(j);j7py(j)];
linksZ=[j1pz(j);j2pz(j);j3pz(j);j4pz(j);j5pz(j);j6pz(j);j7pz(j)];
plot3(linksX,linksY,linksZ,':b','linewidth',4);hold on;        
title('Kinova Simulation Results');
xlabel('X');ylabel('Y');zlabel('Z');
%axis([-0.1,0.5, -0.6,0, 0,0.6]);
grid on;

figure;
plot3(xob,yob,zob,'or','Markersize',10,'linewidth',2);hold on;
plot3(j7px,j7py,j7pz,'r','linewidth',2);hold on;
i=0;
for j=1:length(t)
    if abs(t(j)-i)<=(1e-1)
        EndPosX=[j7px(j);j7px(j)+EndNor(j,1)];
        EndPosY=[j7py(j);j7py(j)+EndNor(j,2)];
        EndPosZ=[j7pz(j);j7pz(j)+EndNor(j,3)];
        plot3(EndPosX,EndPosY,EndPosZ,'k','linewidth',0.1);hold on;
        i = i+(T/15);
    end;
end;
j=1;
EndPosX=[j7px(j);j7px(j)+EndNor(j,1)];
EndPosY=[j7py(j);j7py(j)+EndNor(j,2)];
EndPosZ=[j7pz(j);j7pz(j)+EndNor(j,3)];
plot3(EndPosX,EndPosY,EndPosZ,'b','linewidth',2);hold on;
j=length(t);
EndPosX=[j7px(j);j7px(j)+EndNor(j,1)];
EndPosY=[j7py(j);j7py(j)+EndNor(j,2)];
EndPosZ=[j7pz(j);j7pz(j)+EndNor(j,3)];
plot3(EndPosX,EndPosY,EndPosZ,':b','linewidth',4);hold on;

title('End-effector posistion and posture');
xlabel('X');ylabel('Y');zlabel('Z');
legend('obstacle point','end-effector position','end-effector normal vector','initial normal vector','final normal vector');
grid on;

% figure;
% plot3(xob,yob,zob,'k*');hold on;
% plot3(j7px,j7py,j7pz);hold on;
% for j=1:length(t),
%     joints12=line('xdata',[j1px(j);j2px(j)],'ydata',[j1py(j);j2py(j)],'zdata',[j1pz(j);j2pz(j)],...
%         'color', 'red','erasemode','none');
%     joints23=line('xdata',[j2px(j);j3px(j)],'ydata',[j2py(j);j3py(j)],'zdata',[j2pz(j);j3pz(j)],...
%         'color', 'yellow','erasemode','none');
%     joints34=line('xdata',[j3px(j);j4px(j)],'ydata',[j3py(j);j4py(j)],'zdata',[j3pz(j);j4pz(j)],...
%         'color', 'magenta','erasemode','none');
%     joints45=line('xdata',[j4px(j);j5px(j)],'ydata',[j4py(j);j5py(j)],'zdata',[j4pz(j);j5pz(j)],...
%         'color', 'green','erasemode','none');
%     joints56=line('xdata',[j5px(j);j6px(j)],'ydata',[j5py(j);j6py(j)],'zdata',[j5pz(j);j6pz(j)],...
%         'color', 'blue','erasemode','none');
%     joints67=line('xdata',[j6px(j);j7px(j)],'ydata',[j6py(j);j7py(j)],'zdata',[j6pz(j);j7pz(j)],...
%         'color', 'yellow','erasemode','none');
%     drawnow
% end;
% title('colourful kinova');
% xlabel('X');ylabel('Y');zlabel('Z');
% grid on;
% 
% figure;
% plot3(xob,yob,zob,'k*');hold on;
% plot3(j1px,j1py,j1pz);hold on;
% plot3(j2px,j2py,j2pz);hold on;
% plot3(j3px,j3py,j3pz);hold on;
% plot3(j4px,j4py,j4pz);hold on;
% plot3(j5px,j5py,j5pz);hold on;
% plot3(j6px,j6py,j6pz);hold on;
% plot3(j7px,j7py,j7pz);hold on;
% animate(qAll);
% axis([0,0.45,-0.7,0,0,0.35]);
% axis vis3d
hold on;
disp('5 finished');