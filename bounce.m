function bounce(r)
r.reset
r.turnAngle(30)
nb = 0
v=0.5;
r.setDriveVelocity(v);
while nb<100 
    b = r.getBumpers();
    if b.right
        r.turnAngle(15,v);
        nb=nb+1
        disp('right bump, turn 15');
    elseif b.left
        r.turnAngle(-15,v);
        nb=nb+1
        disp('left bump, turn -15');
    elseif b.front
        r.turnAngle(45,v);
        nb=nb+1
        disp('front bump, turn 45');
    else
        r.setDriveVelocity(v);
    end
end