function u12=my1x3(u1,u2)
for i=0:(length(u1)-1),
    i1=i+1;i31=i*3+1;i13=(i+1)*3;
    u12(i31:i13,1)=u1(i1)*u2(i31:i13,1); 
end;