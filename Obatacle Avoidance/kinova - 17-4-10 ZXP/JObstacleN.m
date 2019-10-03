function [JN,distobf,rightB,effective_oan,distob]=JObstacleN(q,dq)
% ZXP JN=>Jo
% ZXP distobf 小于d2的 critical point 到 obstacle的距离
% ZXP rightB bo
% ZXP effective_oan 与障碍物距离进入d2的连杆数
% ZXP distob critical point 到 obstacle的距离

global lenK d1 d2 xob yob zob oan newB turn D;

% [T01,T12,T23,T34,T45,T56]=TransformN(q,lenK);
% T02=T01*T12;
% T03=T02*T23;
% T04=T03*T34;
% T05=T04*T45;
% T06=T45*T56;
% T07=T06*T67;
[x,y,z,T]=positionN(q,lenK);
% x(1)=0;y(1)=0;z(1)=0;
% x(2)=T01(1,4);y(2)=T01(2,4);z(2)=T01(3,4);
% x(3)=T02(1,4);y(3)=T02(2,4);z(3)=T02(3,4);
% x(4)=T03(1,4);y(4)=T03(2,4);z(4)=T03(3,4);
% x(5)=T04(1,4);y(5)=T04(2,4);z(5)=T04(3,4);
% x(6)=T05(1,4);y(6)=T05(2,4);z(6)=T05(3,4);
% x(7)=T06(1,4);y(7)=T06(2,4);z(7)=T06(3,4);
turn=turn+1;
turn
JN=[];distobf=[];rightB=[];

% ZXP lenob critical point 在连杆上的位置
% ZXP j_ob() critical point 的位置
% ZXP distob critical point距离 obstacle的距离
    for i=2:7, 
        OJs=(xob(1)-x(i-1))^2+(yob(1)-y(i-1))^2+(zob(1)-z(i-1))^2;%s means square  %ZXP ON^2
        OCs=(xob(1)-x(i))^2+(yob(1)-y(i))^2+(zob(1)-z(i))^2;%C: next joint  %ZXP OM^2
        BC=(D(i-1)^2+OCs-OJs)/(2*D(i-1));%OLM
        if BC<=0,
            %case c: at joint i      
            lenob(i-1)=D(i-1);
            jxob(i-1)=x(i); 
            jyob(i-1)=y(i);
            jzob(i-1)=z(i);
            distob(i-1)=OCs^0.5;    %ZXP OM
        elseif BC>=D(i-1),
            %case b: at joint i-1
            lenob(i-1)=0;
            jxob(i-1)=x(i-1);
            jyob(i-1)=y(i-1);
            jzob(i-1)=z(i-1);
            distob(i-1)=OJs^0.5;    %ZXP ON
        else
            %case a: in the link i-1 
            lenob(i-1)=D(i-1)-BC;   %ZXP OLN
            jxob(i-1)=x(i-1)+(x(i)-x(i-1))*lenob(i-1)/D(i-1);
            jyob(i-1)=y(i-1)+(y(i)-y(i-1))*lenob(i-1)/D(i-1);
            jzob(i-1)=z(i-1)+(z(i)-z(i-1))*lenob(i-1)/D(i-1);        
            distob(i-1)=(OCs-BC^2)^0.5; %ZXP OOL
        end;
    end;
    
    
    if (distob(1)<=d2),%previously (distob(1)<=d2), wrong?
        JO=JacobN(q,[lenob(1) 0 0 0 0 0]);
        if (lenob(1)<=d1), disp('too close to the base');end;
        dirob=sign([jxob(1)-xob(1);jyob(1)-yob(1);jzob(1)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(1)];   %ZXP JN即Jo，distobf即d的列向量
        % if distob(1)==d2%(distob(1)==d2)    ZXP distob为浮点数，很难正好等于d2？
        if (abs(distob(1)-d2)<=(10^-3*d2))
        %if abs(distob(1+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2) 
            newB(1,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(1,:)']; %ZXP rightB = [rox1,roy1,roz1,rox2,roy2,roz2...] critical point 的'速度'
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(1),d2),newB(1,:)')];
        end  
    else
        newB(1,:)=zeros(1,3);        
    end;
    if (distob(2)<=d2),
        JO=JacobN(q,[lenK(1) lenob(2) 0 0 0 0]);
        dirob=sign([jxob(2)-xob(1);jyob(2)-yob(1);jzob(2)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(2)];
        if (abs(distob(2)-d2)<=(10^-3*d2))  %(distob(2)==d2) 
        %if abs(distob(2+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2) 
            newB(2,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(2,:)'];
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(2),d2),newB(2,:)')];             
        end 
    else
        newB(2,:)=zeros(1,3);  
    end;
    if (distob(3)<=d2),
        JO=JacobN(q,[lenK(1) lenK(2) lenob(3) 0 0 0]);
        dirob=sign([jxob(3)-xob(1);jyob(3)-yob(1);jzob(3)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(3)];
        if (abs(distob(3)-d2)<=(10^-3*d2))  %(distob(3)==d2) 
%       if abs(distob(3+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2)
            newB(3,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(3,:)'];
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(3),d2),newB(3,:)')];             
        end 
    else
        newB(3,:)=zeros(1,3);  
    end;
    if (distob(4)<=d2),
        JO=JacobN(q,[lenK(1) lenK(2) lenK(3) lenob(4) 0 0]);
%        if (lenK(4)-lenob(4))<=d1, disp('too close to the end');end;
        dirob=sign([jxob(4)-xob(1);jyob(4)-yob(1);jzob(4)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(4)];
        if (abs(distob(4)-d2)<=(10^-3*d2))  %(distob(4)==d2)   
%       if abs(distob(4+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2) 
            newB(4,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(4,:)'];
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(4),d2),newB(4,:)')];             
        end 
    else
        newB(4,:)=zeros(1,3);  
    end;
    if (distob(5)<=d2),
        JO=JacobN(q,[lenK(1) lenK(2) lenK(3) lenK(4) lenob(5) 0]);
%        if (lenK(5)-lenob(5))<=d1, disp('too close to the end');end;
        dirob=sign([jxob(5)-xob(1);jyob(5)-yob(1);jzob(5)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(5)];
        if (abs(distob(5)-d2)<=(10^-3*d2))  %(distob(5)==d2)   
%       if abs(distob(4+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2) 
            newB(5,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(5,:)'];
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(5),d2),newB(5,:)')];             
        end 
    else
        newB(5,:)=zeros(1,3);  
    end;
    if (distob(6)<=d2),
        JO=JacobN(q,[lenK(1) lenK(2) lenK(3) lenK(4) lenK(5) lenob(6)]);
        if (lenK(6)-lenob(6))<=d1, disp('too close to the end');end;
        dirob=sign([jxob(6)-xob(1);jyob(6)-yob(1);jzob(6)-zob(1)]);
        JNew=-[dirob(1)*JO(1,:);dirob(2)*JO(2,:);dirob(3)*JO(3,:)];
        JN=[JN;JNew];distobf=[distobf;distob(6)];
        if (abs(distob(6)-d2)<=(10^-3*d2))  %(distob(1)==d2)   
%       if abs(distob(4+(j-1)*4)-d2)<=(10^-4*d2)%(distob(1)==d2) 
            newB(6,:)=myabs(JNew*dq)';
            rightB=[rightB;newB(6,:)'];
        else
            rightB=[rightB;my1x3(sfunction(d1,distob(6),d2),newB(6,:)')];             
        end 
    else
        newB(6,:)=zeros(1,3);  
    end;

if isempty(JN),
    disp('no obstacle is detected.');
    JN=zeros(3*oan,6);
    rightB=zeros(3*oan,1);
    distobf=min(distob)*ones(oan,1);
    effective_oan=0;
elseif (numrows(JN)>oan*3), %this case will not happen any more
    disp('more than 3+oan*3 neurons are needed! =>3 more neurons');
    JN=JN(1:(oan*3),:);
    rightB=rightB(1:(oan*3),1);
    distobf=distobf(1:oan,1);   
elseif (numrows(JN)<oan*3),
    numJN=numrows(JN);
    effective_oan=numJN/3;
    disp('the following number of neurons are needed:');
    disp(numJN);
    JN((numJN+1):(oan*3),:)=zeros(3*oan-numJN,6);
    rightB((numJN+1):(oan*3),1)=zeros(3*oan-numJN,1);    
    distobf((numJN/3+1):oan,1)=2*d2*ones(oan-numJN/3,1);
else
    disp('wrong?')
end;