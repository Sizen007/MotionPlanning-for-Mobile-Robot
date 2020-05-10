%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;          
x_I=1;                  % ���ó�ʼ��
y_I=1;
x_G=700; y_G=700;       % ����Ŀ���
Thr=80;                 %����Ŀ�����ֵ
Delta= 50;              % ������չ����
%% ������ʼ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;     %
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
x_rand=[];
x_near=[];
V_x = [];
V_y = [];
d = [];
x_new=[];
x_near_index = 0;
count=1;
for iter = 1:2000    
%     x_rand(count,:) = randi([1,700],1,2);%Step 1: �ڵ�ͼ���������һ����x_rand
    x_rand(1) = randi(xL);
    x_rand(2) = randi(yL);
    x_rand_x = x_rand(end,1);        
    x_rand_y = x_rand(end,2);                %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в���������� 
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
    %Step 2: ���������������ҵ�����ڽ���x_near 
    %��ʾ��x_near�Ѿ�����T��
    theta = atan((x_rand_y - x_near(end,2))/(x_rand_x - x_near(end,1)));
    x_new_x = x_near(end,1) + Delta*cos(theta);
    x_new_y = x_near(end,2) + Delta*sin(theta);
    x_new = [x_new_x,x_new_y];                             %Step 3: ��չ�õ�x_new�ڵ�
    
    %���ڵ��Ƿ���collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;
    
    %Step 4: ��x_new������T 
    %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
    T.v(count).x = x_new_x;               % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
    T.v(count).y = x_new_y; 
    T.v(count).xPrev = x_near(end,1);     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
    T.v(count).yPrev = x_near(end,2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = x_near_index;

    %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
    %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
    plot(x_new_x,x_new_y,'yO');
    plot(x_near(end,1),x_near(end,2),'gO');
    plot([x_new_x x_near(end,1)],[x_new_y x_near(end,2)],'-g');
    hold on
    d_end = sqrt((x_new_x-700)^2+(x_new_y-700)^2);
   if d_end < Thr
       disp('Find Path!');
       break       
   end
    
    %Step 6:��x_near��x_new֮���·��������
    %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
    %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
   
   pause(0.1); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 1800
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % �յ����·��
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


