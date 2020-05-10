clear all;
clc;

p_0 = [0,8,20];
v_0 = [];
a_0 = [];
k = 20;
K = 20;
dt = 0.05;
log = [0 0 0 0 0 0 0 0 0 0];

px_0 = 0;
py_0 = 0;%8
pz_0 = 0;%20

vx_0 = 0;
vy_0 = 0;
vz_0 = 0;

ax_0 = 0;
ay_0 = 0;
az_0 = 0;
%% Get prediction trajectory 
for t = dt:dt:10
    %% Reference position
    [pxref,pyref,pzref] = getReferenceposition(k,dt,t);

    %% Construct the prediction matrix
    [Tp_x,Tv_x,Ta_x,Bp_x,Bv_x,Ba_x] = getPredictionMatrix(k,dt,px_0,vx_0,ax_0);
    [Tp_y,Tv_y,Ta_y,Bp_y,Bv_y,Ba_y] = getPredictionMatrix(k,dt,py_0,vy_0,ay_0);
    [Tp_z,Tv_z,Ta_z,Bp_z,Bv_z,Ba_z] = getPredictionMatrix(k,dt,pz_0,vz_0,az_0);
    Tp = blkdiag(Tp_x,Tp_x,Tp_x);
    Tv = blkdiag(Tv_x,Tv_x,Tv_x);
    Ta = blkdiag(Ta_x,Ta_x,Ta_x);
    Bp = [Bp_x;Bp_y;Bp_z];
    Bv = [Bv_x;Bv_y;Bv_z];
    Ba = [Ba_x;Ba_y;Ba_z];
    
    %% Construct the optimization problem
    H = blkdiag(Tp_x'*Tp_x,Tp_y'*Tp_y,Tp_y'*Tp_y);
    F = [Bp_x'*Tp_x-pxref'*Tp_x, Bp_y'*Tp_y-pyref'*Tp_y, Bp_z'*Tp_z-pzref'*Tp_z];
    
    A = [Tv;-Tv;Ta;-Ta];
    b = [6*ones(20,1)-Bv_x;6*ones(20,1)+Bv_x;6*ones(20,1)-Bv_y;6*ones(20,1)+Bv_y;6*ones(20,1)-Bv_y;ones(20,1)+Bv_y;3*ones(20,1)-Ba_x;3*ones(20,1)+Ba_x;3*ones(20,1)-Ba_y;3*ones(20,1)+Ba_y;3*ones(20,1)-Ba_y;ones(20,1)+Ba_y];
    
    %% Solve the optimization problem
    %J = quadprog(H,F,A,b);
    J = quadprog(H,F,[],[]);

    %% Apply the control
    jx = J(1);
    jy = J(k+1);
    jz = J(2*k+1);
    
    px_0 = px_0 + vx_0*dt + 0.5*ax_0*dt^2 + 1/6*jx*dt^3;
    vx_0 = vx_0 + ax_0*dt + 0.5*jx*dt^2;
    ax_0 = ax_0 + jx*dt;
    
    py_0 = py_0 + vy_0*dt + 0.5*ay_0*dt^2 + 1/6*jy*dt^3;
    vy_0 = vy_0 + ay_0*dt + 0.5*jy*dt^2;
    ay_0 = ay_0 + jy*dt;
    
    pz_0 = pz_0 + vz_0*dt + 0.5*az_0*dt^2 + 1/6*jz*dt^3;
    vz_0 = vz_0 + az_0*dt + 0.5*jz*dt^2;
    az_0 = az_0 + jz*dt;
    
    log = [log; t px_0 vx_0 ax_0 py_0 vy_0 ay_0 pz_0 vz_0 az_0];  
end

%% Get reference trajectory
K_disp = 10/dt;
w = 0.08/pi*180;
v = -0.5;
x = zeros(K_disp+1,1);
y = zeros(K_disp+1,1);
z = zeros(K_disp+1,1);
for i = 2:K_disp+1
    x(i,1) = v/2*(i*dt)*cos(w*i*dt) ;
    y(i,1) = v/2*(i*dt)*sin(w*i*dt) ;
    z(i,1) = v*(i*dt) ;
end
% x = [0;x];
% y = [4;y];
% z = [0;z];

%% Plot predict and reference trajectory
figure;
plot3(x,y,z,'.','Color','g');hold on;
plot3(log(:,2),log(:,5),log(:,8),'b');hold on;



