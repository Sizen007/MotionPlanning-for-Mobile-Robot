%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all;          
x_I=1;                  % 设置初始点
y_I=1;
x_G=700; y_G=700;       % 设置目标点
Thr=80;                 %设置目标点阈值
Delta= 50;              % 设置扩展步长
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
x_rand=[];
x_near=[];
V_x = [];
V_y = [];
d = [];
x_new=[];
x_near_index = 0;
count=1;
for iter = 1:2000    
%     x_rand(count,:) = randi([1,700],1,2);%Step 1: 在地图中随机采样一个点x_rand
    x_rand(1) = randi(xL);
    x_rand(2) = randi(yL);
    x_rand_x = x_rand(end,1);        
    x_rand_y = x_rand(end,2);                %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标 
    V_x = [T.v.x];
    V_y = [T.v.y];
    Len_V = length(V_x(1,:));
    if Len_V == 1
        x_near = [1,1];
        x_near_index = 1;
%          T.v(count).indPrev = 1;
    else
        for i = 1:Len_V 
            d(i)=sqrt((x_rand_x-V_x(i))^2+(x_rand_y-V_y(i))^2);    
        end
        [m,p] = min(d);
        x_near = [V_x(p),V_y(p)];
        x_near_index = p;
    end
    %Step 2: 遍历树，从树中找到最近邻近点x_near 
    %提示：x_near已经在树T里
    theta = atan((x_rand_y - x_near(end,2))/(x_rand_x - x_near(end,1)));
    x_new_x = x_near(end,1) + Delta*cos(theta);
    x_new_y = x_near(end,2) + Delta*sin(theta);
    x_new = [x_new_x,x_new_y];                             %Step 3: 扩展得到x_new节点
    
    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;
    
    %Step 4: 将x_new插入树T 
    %提示：新节点x_new的父节点是x_near
    T.v(count).x = x_new_x;               % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
    T.v(count).y = x_new_y; 
    T.v(count).xPrev = x_near(end,1);     % 起始节点的父节点仍然是其本身
    T.v(count).yPrev = x_near(end,2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = x_near_index;

    %Step 5:检查是否到达目标点附近 
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    plot(x_new_x,x_new_y,'yO');
    plot(x_near(end,1),x_near(end,2),'gO');
    plot([x_new_x x_near(end,1)],[x_new_y x_near(end,2)],'-g');
    hold on
    d_end = sqrt((x_new_x-700)^2+(x_new_y-700)^2);
   if d_end < Thr
       disp('Find Path!');
       break       
   end
    
    %Step 6:将x_near和x_new之间的路径画出来
    %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
    %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来
   
   pause(0.1); %暂停0.1s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 1800
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


