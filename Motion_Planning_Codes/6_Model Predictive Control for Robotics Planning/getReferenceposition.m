function [pxref,pyref,pzref] = getReferenceposition(k,dt,t)
w = 0.08/pi*180;
v = -0.5;
r = 10;
h = 20;
pxref = zeros(k,1);
pyref = zeros(k,1);
pzref = zeros(k,1);
start = ceil(t/dt);

for i = start:k+start-1  
    pxref(i-start+1,1) = v/2*(i*dt)*cos(w*i*dt);
    pyref(i-start+1,1) = v/2*(i*dt)*sin(w*i*dt);
    pzref(i-start+1,1) = v*(i*dt);

end

