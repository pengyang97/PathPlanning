%% ��ջ���
clc;
clear all;
close all;
tic
%% �ϰ�������
position = load('barrier.txt');
plot([0,200],[0,200],'.');
hold on
B = load('barrier.txt');
xlabel('km','fontsize',12)
ylabel('km','fontsize',12)
title('��ά�滮�ռ�','fontsize',12)
%% ���������յ�
Start = [20,180];
Target = [160,90];
plot([Start(1),Target(1)],[Start(2),Target(2)],'.');

% ͼ�α�ע
text(Start(1),Start(2)+7,'S');
text(Target(1)+3,Target(2)-4,'T');
 
%% ����ϰ���ͼ��
fill(position(1:4,1),position(1:4,2),[0,0,0]);
fill(position(5:8,1),position(5:8,2),[0,0,0]);
fill(position(9:12,1),position(9:12,2),[0,0,0]);
fill(position(13:15,1),position(13:15,2),[0,0,0]);

% ������·�˵�����
L = load('lines.txt');
 
%% ����߼��е�
v = zeros(size(L));
for i=1:20
    plot([position(L(i,1),1),position(L(i,2),1)],[position(L(i,1),2)...
        ,position(L(i,2),2)],'color','black','LineStyle','--');
    v(i,:) = (position(L(i,1),:)+position(L(i,2),:))/2;
    plot(v(i,1),v(i,2),'*');
    text(v(i,1)+2,v(i,2),strcat('v',num2str(i)));
end

%% ������·��
sign = load('matrix.txt');
[n,m]=size(sign);
 
for i=1:n
    
    if i == 1
        for k=1:m-1
            if sign(i,k) == 1
                plot([Start(1),v(k-1,1)],[Start(2),v(k-1,2)],'color',...
                    'black','Linewidth',2,'LineStyle','-');
            end
        end
        continue;
    end
    
    for j=2:i
        if i == m
            if sign(i,j) == 1
                plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color',...
                    'black','Linewidth',2,'LineStyle','-');
            end
        else
            if sign(i,j) == 1
                plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],...
                    'color','black','Linewidth',2,'LineStyle','-');
            end
        end
    end
end                                       %�����ܻ���MAKLINK��·ͼ
v1=zeros(22,2);                           %v1��Ű���S��T���ܹ�22����㣬��ΪDijkstraPlan�Ĳ���
v1(1,:)=[20,180];
v1(22,:)=[160,90];
for i=2:21
    v1(i,:)=v(i-1,:);
end
path = DijkstraPlan(v1,sign);
j = path(22);
plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color','yellow','LineWidth',3,'LineStyle','-.');
i = path(22);
j = path(i);
count = 0;
while true
    plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],'color','yellow','LineWidth',3,'LineStyle','-.');
    count = count + 1;
    i = j;
    j = path(i);
    if i == 1 || j==1
        break;
    end
end
plot([Start(1),v(i-1,1)],[Start(2),v(i-1,2)],'color','yellow','LineWidth',3,'LineStyle','-.');
count = count+3;
pathtemp(count) = 22;
j = 22;
for i=2:count
    pathtemp(count-i+1) = path(j);
    j = path(j);
end
path = pathtemp;                     %�������ڹ滮�ռ����û��������������·��
pathCount = length(path)-2;          %�����߶��������൱��ά��D

%% ������������
lines = zeros(pathCount,4);
for i = 1:pathCount                  %lines���ڴ�Ÿ���������������ߵ������յ�
    lines(i,1:2) = B(L(path(i+1)-1,1),:);
    lines(i,3:4) = B(L(path(i+1)-1,2),:);
end

%% ��ʼ���·��
dijpathlen = 0;
vv = zeros(22,2);
vv(1,:) = Start;
vv(22,:) = Target;
vv(2:21,:) = v;
for i=1:pathCount+1
dijpathlen = dijpathlen + sqrt((vv(path(i),1)-vv(path(i+1),1))^2+(vv(path(i),2)-vv(path(i+1),2))^2);
end
LL = dijpathlen;                   %�ó���LLΪ�������·���ĳ���

%% �����㷨������ʼ��
popsize=30;
maxgen=200;
fmin=0;
fmax=10;
r0=0.5;
A=10;
gen=0;
bestgen=zeros(1,maxgen);

%% ��ʼ��
X=rand(pathCount,popsize);
d=zeros(1,popsize);
w=zeros(pathCount,1);
for i=1:m
    w=X(:,i);
    fitness(i)=lenf(w,pathCount,lines);
end
[ww,i]=min(fitness);
minX=X(:,i);
minY=ww;                             %��ʼ������¼
besty=zeros(1,NC);                   %����壬�������ű�����ԣ�������ʷ���Ÿ���
shortestpath = zeros(1,NC);          %�������̼�¼
besty(1)=minY;
shortestpath(1)=minY;

% % �����㷨���й���
% while(gen<maxgen)
%     for i=1:popsize
%         
% end






t_train=toc;             %������ʱ
%����������е����·������Ӧ���˹���Ĺ켣
lastline=zeros(pathCount+2,2);
lastline(1,:)=Start;
lastline(pathCount+2,:)=Target;
pathkbest=bestS;
for i=1:pathCount
    lastline(i+1,:)=lines(i,1:2) + (lines(i,3:4)-lines(i,1:2))*pathkbest(:,i);
end

for i=1:pathCount+1
    plot([lastline(i,1),lastline(i+1,1)],[lastline(i,2),lastline(i+1,2)],'color','red','LineWidth',3);
end
%������·������������ı仯
figure(2);
% plot(bestgensmell,'color','red');
plot(bestgensmell);
hold on
title( ['�������еõ�������ֵΪ',num2str(SmellBest),',�ܹ���ʱ',num2str(t_train),'s'] );
%text(180,190,'����ֵ����������仯����');
ylabel('·���ܳ���');
xlabel('��������');
