function sd=sfunction(d1,d,d2)
for i=1:length(d),
    if (d(i)>=d2) 
        sd(i,1)=1;
    elseif (d(i)<=d1) 
        sd(i,1)=0;
    else 
        sd(i,1)=(d(i)-d1)/(d2-d1);%这是03的做法，04j用了别的函数
    end;
end;