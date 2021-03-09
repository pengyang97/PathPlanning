%% ��ջ���
clc;clear all
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
S = [20,180];
T = [160,90];
plot([S(1),T(1)],[S(2),T(2)],'.');

% ͼ�α�ע
text(S(1),S(2)+7,'S');
text(T(1)+3,T(2)-4,'T');
 
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
                plot([S(1),v(k-1,1)],[S(2),v(k-1,2)],'color',...
                    'black','Linewidth',2,'LineStyle','-');
            end
        end
        continue;
    end
    
    for j=2:i
        if i == m
            if sign(i,j) == 1
                plot([T(1),v(j-1,1)],[T(2),v(j-1,2)],'color',...
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
plot([T(1),v(j-1,1)],[T(2),v(j-1,2)],'color','yellow','LineWidth',3,'LineStyle','-.');
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
plot([S(1),v(i-1,1)],[S(2),v(i-1,2)],'color','yellow','LineWidth',3,'LineStyle','-.');


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
vv(1,:) = S;
vv(22,:) = T;
vv(2:21,:) = v;
for i=1:pathCount+1
dijpathlen = dijpathlen + sqrt((vv(path(i),1)-vv(path(i+1),1))^2+(vv(path(i),2)-vv(path(i+1),2))^2);
end
LL = dijpathlen;                   %�ó���LLΪ�������·���ĳ���

%% �˹���Ⱥ�㷨������ʼ��
Visual=0.25;
Step=0.03;
delta=10;
Try_number=6;                        %��ʳʱ�����̽����
a=0;                                 %x����
b=1;                                 %x����
m=30;                                %�˹�������
NC=300;                              %ѭ����������
X=rand(pathCount,m);                 %��ʼ���˹���Ⱥ
d=zeros(1,m);
w=zeros(pathCount,1);
for i=1:m
    w=X(:,i);
    d(i)=lenf(w,pathCount,lines);
end
[ww,i]=min(d);
minX=X(:,i);
minY=ww;                             %��ʼ������¼
besty=zeros(1,NC);                   %����壬�������ű�����ԣ�������ʷ���Ÿ���
shortestpath = zeros(1,NC);          %�������̼�¼
besty(1)=minY;
shortestpath(1)=minY;
%% ѭ������
for num = 1:NC-1
    for i=1:m
        XX=X( : ,i);                 %�˹��㵱ǰ״̬   
        nf=0;
        Xc=zeros(pathCount,1);
      for j=1:m                
          XX1=X( : ,j);
          if( norm(XX1-XX)<Visual )
              nf=nf+1;
              Xc=Xc+XX1;
          end
      end
          Xc=Xc/nf;
          if( lenf(Xc,pathCount,lines)*nf < lenf(XX,pathCount,lines)*delta & Xc~=XX )
              XXnext1=XX+rand*Step*(Xc-XX)/norm(Xc-XX);
           for k=1:pathCount
              if XXnext1(k,1)>b
                  XXnext1(k,1)=b;
              end
              if XXnext1(k,1)<a
                  XXnext1(k,1)=a;
              end
           end
          else
             XXnext1=gmjprey(XX,Try_number,Visual,Step,pathCount,lines);   %ִ����ʳ��Ϊ
             for k=1:pathCount
              if XXnext1(k,1)>b
                  XXnext1(k,1)=b;
              end
              if XXnext1(k,1)<a
                  XXnext1(k,1)=a;
              end
             end      
          end                          %��Ⱥ��Ϊ����
          
        nf=0;                          %׷β��Ϊ��ʼ
        miny=3000; 
        for l=1:m                      %�ҳ�XX�����ڵ���С����
          XX2=X(:,l);
          if( norm(XX2-XX)<Visual && lenf(XX2,pathCount,lines)<miny )
              minx=XX2;
              miny=lenf(XX2,pathCount,lines);
          end
        end
        
        for t=1:m                      %�ҳ�����С����minx�����ڵ��˹������nf
          XX3=X( : ,t);
          if( norm(XX3-minx) < Visual )
              nf=nf+1;
          end
        end
      
          if( miny*nf < lenf(XX,pathCount,lines)*delta & minx~=XX ) %�жϸ���С����minx�Ƿ�ӵ��
              XXnext2=XX+rand*Step*(minx-XX)/norm(minx-XX);
              for p=1:pathCount
               if XXnext2(p,1)>b
                   XXnext2(p,1)=b;
               end
               if XXnext2(p,1)<a
                   XXnext2(p,1)=a;
               end
              end
          else
             XXnext2=gmjprey(XX,Try_number,Visual,Step,pathCount,lines);   %ִ����ʳ��Ϊ
             for q=1:pathCount
              if XXnext2(q,1)>b
                  XXnext2(q,1)=b;
              end
              if XXnext2(q,1)<a
                  XXnext2(q,1)=a;
              end
             end 
          end
             if(lenf(XXnext1,pathCount,lines) < lenf(XXnext2,pathCount,lines))
               X(:,i)=XXnext1;
             else
               X(:,i)=XXnext2;
             end
           if( lenf(X(:,i),pathCount,lines) < minY )
               minX=X(:,i);
               minY=lenf(X(:,i),pathCount,lines);
           end
    end        %һ�ε�������
    besty(num+1) = minY;            
    len = zeros(1,m);
    for i=1:m
        len(i) = lenf(X(:,i),pathCount,lines);
    end
    %Ѱ�����·��
    minlen = min(len);
    shortestpath(num+1) = minlen;
end
t_train=toc;             %������ʱ
%����������е����·������Ӧ���˹���Ĺ켣
lastline=zeros(pathCount+2,2);
lastline(1,:)=S;
lastline(pathCount+2,:)=T;
pathkbest=minX;
for i=1:pathCount
    lastline(i+1,:)=lines(i,1:2) + (lines(i,3:4)-lines(i,1:2))*pathkbest(i,:);
end

for i=1:pathCount+1
    plot([lastline(i,1),lastline(i+1,1)],[lastline(i,2),lastline(i+1,2)],'color','red','LineWidth',3);
end
%������·������������ı仯
figure;
%plot(shortestpath,'color','blue');
%plot(1:NC,dijpathlen,'color','red');
plot(besty,'color','red');
hold on
title( ['�������еõ�������ֵΪ',num2str(minY),',�ܹ���ʱ',num2str(t_train),'s'] );
%text(180,190,'����ֵ����������仯����');
ylabel('·���ܳ���');
xlabel('��������');
















