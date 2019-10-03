function gf=myabs(u)
for i=1:length(u),
    if (u(i)>=0) 
        gf(i,1)=u(i);
    else 
        gf(i,1)=0;
    end;
end;