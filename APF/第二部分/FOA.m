%% ��ջ���
clc;
clear all;
close all;
tic
%% �ϰ�������
position = load('barrier.txt');
% plot([0,100],[0,100],'.');
axis([0 100 0 100]);
hold on
B = load('barrier.txt');
xlabel('km','fontsize',12)
ylabel('km','fontsize',12)
title('��ά�滮�ռ�','fontsize',12)
%% ���������յ�
Start = [10,90];
Target = [80,45];
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
v = zeros(size(L)); %�洢�е������
for i=1:20   %%20��ʾ������Ŀ���е���Ŀ
    plot([position(L(i,1),1),position(L(i,2),1)],[position(L(i,1),2)...
        ,position(L(i,2),2)],'color','black','LineStyle','--');
    v(i,:) = (position(L(i,1),:)+position(L(i,2),:))/2;
    plot(v(i,1),v(i,2),'*');
    text(v(i,1)+2,v(i,2),strcat('v',num2str(i)));
end

%% ������·��
sign = load('matrix.txt');%�ڽӾ���
[n,m]=size(sign);

for i=1:n
    
    if i == 1
        for k=1:m-1
            if sign(i,k) == 1
                plot([Start(1),v(k-1,1)],[Start(2),v(k-1,2)],'color',...
                    'black','Linewidth',1,'LineStyle','-');
            end
        end
        continue;
    end
    
    for j=2:i
        if i == m
            if sign(i,j) == 1
                plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color',...
                    'black','Linewidth',1,'LineStyle','-');
            end
        else
            if sign(i,j) == 1
                plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],...
                    'color','black','Linewidth',1,'LineStyle','-');
            end
        end
    end
end                                                                  %�����ܻ���MAKLINK��·ͼ
v1=zeros(22,2);  %20���е������ʼ�����ֹ��                         %v1��Ű���S��T���ܹ�22����㣬��ΪDijkstraPlan�Ĳ���
v1(1,:)=[20,180];
v1(22,:)=[160,90];
for i=2:21
    v1(i,:)=v(i-1,:);
end
path = DijkstraPlan(v1,sign);
j = path(22);
plot([Target(1),v(j-1,1)],[Target(2),v(j-1,2)],'color','black','LineWidth',3,'LineStyle',':');
i = path(22);
j = path(i);
count = 0;
while true
    plot([v(i-1,1),v(j-1,1)],[v(i-1,2),v(j-1,2)],'color','black','LineWidth',3,'LineStyle',':');
    count = count + 1;
    i = j;
    j = path(i);
    if i == 1 || j==1
        break;
    end
end
plot([Start(1),v(i-1,1)],[Start(2),v(i-1,2)],'color','black','LineWidth',3,'LineStyle',':');
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

% ��ʼ���·��
dijpathlen = 0;
vv = zeros(22,2);
vv(1,:) = Start;
vv(22,:) = Target;
vv(2:21,:) = v;
for i=1:pathCount+1
dijpathlen = dijpathlen + sqrt((vv(path(i),1)-vv(path(i+1),1))^2+(vv(path(i),2)-vv(path(i+1),2))^2);
end
LL = dijpathlen;                   %�ó���LLΪ�������·���ĳ���

%% ��Ӭ�Ż��㷨������ʼ��
popsize=50;
maxgen=200;
gen=0;
D=zeros(popsize,pathCount);
S=zeros(popsize,pathCount);
bestgensmell=zeros(1,maxgen);

%% ��Ӭλ�ó�ʼ����ʳ��Ũ�ȼ���
%*** �����ʼ��ӬȺ��λ�á�
X_axis=1+1*rands(1,pathCount);
Y_axis=1+1*rands(1,pathCount);

% *** ��ӬѰ�ſ�ʼ���������Ѱ��ʳ�
for p=1:popsize
    X(p,:)=X_axis+2*rand()-1;
    Y(p,:)=Y_axis+2*rand()-1;

    for i=1:pathCount
    %*** �����ԭ��֮����
%     D(p,i)=(X(p,i)^2+Y(p,i)^2)^0.5;
%     %*** ζ��Ũ��Ϊ����֮�����������ζ��Ũ���ж�ֵ��
%     S(p,i)=1/D(p,i);
    S(p,i)=(X(p,i)+Y(p,i))*1.0/2;
    %%%%%%
    if(S(p,i)>1)%%��Χ�޶���0��1֮��
            S(p,i)=1;
    end
        if(S(p,i)<0)
            S(p,i)=0;
        end

    end
    
end


    %*** ����ζ��Ũ���ж��������ζ��Ũ��
for p=1:popsize
    w=S(p,:);
    Smell(p)=distance(w,pathCount,lines,Start,Target);
end

%% ��ʼ�������
[bestsmell, bestindex]=min(Smell);
%*** �����Ӿ�Ѱ�һ��ۼ�ζ��Ũ�����֮���������Ǳ������ֵ��ʼλ�ü���ʼζ��Ũ�ȡ�
X_axis=X(bestindex,:);
Y_axis=Y(bestindex,:);
bestS=S(bestindex,:);
SmellBest=bestsmell;

%% ��Ӭ��������
%*** ��Ӭ����Ѱ��
for gen=1:maxgen
    %*** �������Ѱ��ʳ��
    for p=1:popsize
    %*** ��ʼ��Ӭ������о���
    X(p,:)=X_axis+2*rand()-1;
    Y(p,:)=Y_axis+2*rand()-1;

    for i=1:pathCount
    %*** �����ԭ��֮����
%     D(p,i)=(X(p,i)^2+Y(p,i)^2)^0.5;
%     %*** ζ��Ũ��Ϊ����֮�����������ζ��Ũ���ж�ֵ��
%     S(p,i)=1/D(p,i);
    S(p,i)=(X(p,i)+Y(p,i))*1.0/2;
    %%%%%%
    if(S(p,i)>1)%%��Χ�޶���0��1֮��
            S(p,i)=1;
    end
        if(S(p,i)<0)
            S(p,i)=0;
        end
    end
    
    end

    %*** ����ζ��Ũ���ж��������ζ��Ũ��
    for p=1:popsize
        w=S(p,:);
        Smell(p)=distance(w,pathCount,lines,Start,Target);
    end
    [bestsmell, bestindex]=min(Smell);
    
    %*** �����Ӿ�Ѱ�һ��ۼ�ζ��Ũ�����֮���������Ǳ������ֵ��ʼλ�ü���ʼζ��Ũ��
    if bestsmell<SmellBest
    X_axis=X(bestindex,:);
    Y_axis=Y(bestindex,:);
    bestS=S(bestindex,:);
    SmellBest=bestsmell;
    end
    bestgensmell(1,gen)=SmellBest;
    yy(gen)=SmellBest;
    Xbest(gen,:)=X_axis;
    Ybest(gen,:)=Y_axis;
end
t_train=toc;             %������ʱ
%����������е����·���켣
lastline=zeros(pathCount+2,2);
lastline(1,:)=Start;
lastline(pathCount+2,:)=Target;
pathkbest=bestS;
for i=1:pathCount
    lastline(i+1,:)=lines(i,1:2) + (lines(i,3:4)-lines(i,1:2))*pathkbest(:,i);
end

for i=1:pathCount+1
    plot([lastline(i,1),lastline(i+1,1)],[lastline(i,2),lastline(i+1,2)],'color','black','LineWidth',3);
end
%������·������������ı仯
% figure(2);
% % plot(bestgensmell,'color','red');
% plot(bestgensmell);
% hold on
% title( ['�������еõ�������ֵΪ',num2str(SmellBest),',�ܹ���ʱ',num2str(t_train),'s'] );
% %text(180,190,'����ֵ����������仯����');
% ylabel('·���ܳ���');
% xlabel('��������');
% figure(3);
% plot(Xbest,Ybest,'b.');
% title('Fruit fly flying route','fontsize',14)
% xlabel('X-axis','fontsize',12);ylabel('Y-axis','fontsize',12);
