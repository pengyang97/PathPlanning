%***************************************
%Author: Yuncheng JIANG
%***************************************
%% ���̳�ʼ��
clc; 
clear all
close all
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=700; y_G=700;       % ����Ŀ���
Thr=50;                 %����Ŀ�����ֵ
delta = 0.35 ;   % ������չ����
iteration = 10000;  % ��������
%% ������ʼ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;     %���ڵ������
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('2Dmap_RRT.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
hold on
count=1;
j =1;
for iter = 1:iteration
    iter    %��ʾ����������
    x_rand(1) = rand(1)*800;    %����������x����
    x_rand(2) = rand(1)*800;    %����������y����
    x_rand = [x_rand(1),x_rand(2)];
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    
    dist =zeros(length(T),1);   %��һ������������¼���Ͻڵ㵽�����֮��ľ���
    
    for i =1: count
        dist(i) = (T.v(i).x- x_rand(1))^2 + (T.v(i).y - x_rand(2))^2;   %�򻯼���Ͳ�ƽ����
        [value, location] = min(dist);
        x_near = [T.v(location).x,T.v(location).y];
    end
    %Step 2: �������е�ÿ���ڵ㣬�������ҵ�����ڽ���x_near 
    %��ʾ��x_near�Ѿ�����T��
 
    x_new=[x_near(1)+delta * (x_rand(1)-x_near(1)), x_near(2)+ delta * (x_rand(2)-x_near(2))];
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
    
    %���ڵ��Ƿ���collision-free
    if ~collisionChecking(x_near,x_new,Imp) 
      
       continue;    %��������ײ���������
    end
    count=count+1;   
   
    pos =count;
    %Step 4: ��x_new������T 
    %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
    T.v(pos).x = x_new(1);         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
    T.v(pos).y = x_new(2); 
    T.v(pos).xPrev = x_near(1);     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
    T.v(pos).yPrev = x_near(2);
    T.v(pos).dist=sqrt((x_new(1)-x_near(1))^2 + (x_new(2)-x_near(2))^2);    %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
    T.v(pos).indPrev = T.v(pos-1).indPrev + 1;
    %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
    %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
    epsilon =10;
   %Step 6:��x_near��x_new֮���·��������
   %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
   %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
   plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r')
   hold on
   pause(0.1); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
      if sqrt((x_G-x_new(1))^2 + (y_G - x_new(2))^2)<=Thr
       break;
      end
      
end
%% ·���Ѿ��ҵ��������ѯ(��δ���ʲô���о�ûë�������ǳ���)
if iter < iteration
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
    path.pos(pos+1).x = x_I; path.pos(pos+1).y = y_I; % ������·��
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
%%
% path(1,:) =[T.v(pos).x, T.v(pos).y];  %��������ɵľ���Ŀ�������ĵ���Ϊ�������·���ĵ�һ����
% k =1;
%  pin =[T.v(pos).xPrev, T.v(pos).yPrev];   %��һ����ĸ��ڵ�
%  condi = 0;
%  while condi ==0
%   %�����������ɵĽڵ����η��򽫵�����ʼ��ĵ���ӵ�·����
% for i = 1:pos
%    i
%     if T.v(i).x == pin(1) && T.v(i).y ==pin(2)
%         path(k+1,:)= [pin(1), pin(2)];
%          k = k+1;
%          pin =[T.v(i).xPrev, T.v(i).yPrev];
%        
%     end
% end
%   if pin(1)== T.v(1).x &&pin(2) ==T.v(1).y
%         condi =1; 
%   end
%    path = [path;[x_I,y_I]];   
%  end
% plot(path(:,1),path(:,2),'b', 'Linewidth', 3) %��������ҵĽڵ���������
% plot([path(1,1),x_G],[path(1,2),y_G],'b', 'Linewidth', 3) %��Ŀ��������һ���ڵ���������



