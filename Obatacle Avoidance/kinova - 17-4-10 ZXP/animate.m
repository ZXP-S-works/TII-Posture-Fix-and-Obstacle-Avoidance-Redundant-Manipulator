function animate(tg)
    global lenK;
    %VERSION of April 15th, 2001
	np=numrows(tg);
	n=6;
    erasemode='xor';
	reach=0.315;
    figure(gcf);%bring to the top
    grid on
	title('PA10+');
	xlabel('X');ylabel('Y');zlabel('Z')
	set(gca,'drawmode','fast');%get handle of current axes, but ?
	line('xdata', [0;0], 'ydata', [0;0], 'zdata', [-reach;0], 'color', 'magenta');
	
    %create a line which we will subsequently modify. Set erase mode to xor for fast update
	hr=line('color', 'black', 'erasemode', erasemode);
	hx=line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;0], ...
		'color', 'red', 'erasemode', 'xor');
	hy=line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;0], ...
		'color', 'green', 'erasemode', 'xor');
	hz=line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;0], ...
		'color', 'blue', 'erasemode', 'xor');
    mag=0.2*reach;
    
	for repeat=1:10000000,
        for p=1:np,
            [x,y,z,t]=positionN(tg(p,:),lenK);
            %compute the wrist axes
            xv=t*[mag;0;0;1];
            yv=t*[0;mag;0;1];
            zv=t*[0;0;mag;1];
            %update the line segments, wrist axis and links
            set(hx,'xdata',[t(1,4) xv(1)], 'ydata', [t(2,4) xv(2)], ...
                'zdata', [t(3,4) xv(3)]);
            set(hy,'xdata',[t(1,4) yv(1)], 'ydata', [t(2,4) yv(2)], ...
                'zdata', [t(3,4) yv(3)]);
            set(hz,'xdata',[t(1,4) zv(1)], 'ydata', [t(2,4) zv(2)], ...
                'zdata', [t(3,4) zv(3)]);
            set(hr,'xdata',x,'ydata',y,'zdata', z);
            drawnow        
            for ii=1:10000,iii=ii^2;end;
        end
    end