function keydrive(rob)
h=figure(2);
set(h,'WindowKeyPressFcn',@dostuff)
set(h,'UserData',rob)
set(h,'units','normalized','position',[.6,.6,.3,.3]);
axis off
text(0.5,3,{'Drive: arrow keys','Stop: space','Beep: 1-8','Snapshot: P'});
axis([0,0,3,3]);
% t = timer;
% t.Ex
ecutionMode= 'fixedSpacing';
% t.TasksToExecute= inf;
% t.Period= 1;
% t.TimerFcn= {@timerfcn,rob};
% rob.userData.counter=0;
% start(t);
end

function timerfcn(h,e,rob)
c = mod(rob.userData.counter + 1,6);
rob.userData.counter=c;
v=2^c;
rob.setLEDDigitsRaw([v v v v]);
end

function dostuff(h,keydata)
rob = get(h,'UserData');
notes.k1 = 'c/2';
notes.k2 = 'd/2';
notes.k3 = 'e/2';
notes.k4 = 'f/2';
notes.k5 = 'g/2';
notes.k6 = 'a/2';
notes.k7 = 'b/2';
notes.k8 = 'c^/2';

try
    v = rob.userData.vel;
    d = rob.userData.dir;
catch
    v = 0.1;
    d = 'space';
end
dv=0.05;
k = keydata.Key;
switch k
    case 'f'
        v = min(v+dv,0.5);
        k = d;
    case 's'
        v = max(v-dv,0.05);
        k = d;
end
switch k
    case 'uparrow'
        rob.setDriveVelocity(v,v);
        rob.userData.dir = k;
    case 'downarrow'
        rob.setDriveVelocity(-v,-v);
        rob.userData.dir = k;
    case 'leftarrow'
        rob.setDriveVelocity(v/2,-v/2);
        rob.userData.dir = k;
    case 'rightarrow'
        rob.setDriveVelocity(-v/2,v/2);
        rob.userData.dir = k;
    case 'space'
        rob.stop;
        rob.userData.dir = k;
    case {'1','2','3','4','5','6','7','8'}
        rob.songPlay(notes.(['k' keydata.Key]),0);
    case 'q'
        rob.stop
        close(2);
    case 'p'
        imshow(rob.getImage)
    otherwise
        disp(['no key handler for ', keydata.Key])
end
rob.userData.vel=v;
end