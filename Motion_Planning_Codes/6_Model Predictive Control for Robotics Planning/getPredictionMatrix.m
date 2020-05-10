function [Tp,Tv,Ta,Bp,Bv,Ba] =getPredictionMatrix(k,dt,p_0,v_0,a_0)
Ta = zeros(k);
Tv = zeros(k);
Tp = zeros(k);

for i = 1:k
    Ta(i,1:i) = ones(1,i)*dt;
end

for i = 1:k
    for j = 1:i
        Tv(i,j) = (i-j+0.5)*dt^2; 
    end
end

for i = 1:k
    for j = 1:i
        Tp(i,j) = ((i-j+1)*(i-j)/2+1/6)*dt^3; 
    end
end

Ba = ones(k,1)*a_0;
Bv = ones(k,1)*v_0;
Bp = ones(k,1)*p_0;

for i = 1:k
    Bv(i) = Bv(i) + i*dt*a_0;
    Bp(i) = Bp(i) + i*dt*v_0 + i^2/2*a_0*dt^2;   
end

end

