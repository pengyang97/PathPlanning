clc,clear all 
%��ʼ�����Ĳ���
Xo=[0 0];%���λ��
k=15;%����������Ҫ������ϵ��
K=0;%��ʼ��
m=16;%�������������ϵ���������Լ��趨�ġ�
Po=15;%�ϰ�Ӱ����룬���ϰ��ͳ��ľ�������������ʱ������Ϊ0�������ܸ��ϰ���Ӱ�졣Ҳ���Լ��趨��
%d_goal = 80;%������Ӱ������һ����ֵ����������һ������������ֵ��Ϊ�˷�ֹ����������Ŀ����Զʱ�������󣬵��³���ײ���ϰ���
d_goal = 50;%������Ӱ������һ����ֵ����������һ������������ֵ��Ϊ�˷�ֹ����������Ŀ����Զʱ�������󣬵��³���ײ���ϰ���
b_goal = 15;%�˷�Ŀ�����
n=15;%�ϰ�����
a=4;%�����a=5�൱�ڸĽ��������ᵽ��n���е�������������Ϊ��2
l=2;%����
J=200;%ѭ����������
%�������ʵ��Ԥ��Ŀ�꣬����Ҳ���ʼ������ϵ����Po���õĲ������йء�
%end
%�����ϰ���Ŀ����Ϣ
% a=10;n=15;%�Լ���
p=unifrnd(0,100,n,2);%n����2���������е����������꣬���ȷֲ���Ӧ����15��2�е�һ����ά���飬���������������ϰ��������꣩

X_rand = p(:,1);%�ϰ�����X��������
Y_rand = p(:,2);%�ϰ�����Y��������
Xsum=[100 100;X_rand(1) Y_rand(1);X_rand(2) Y_rand(2);X_rand(3) Y_rand(3);X_rand(4) Y_rand(4);X_rand(5) Y_rand(5);X_rand(6) Y_rand(6);X_rand(7) Y_rand(7);...
    X_rand(8) Y_rand(8);X_rand(9) Y_rand(9);X_rand(10) Y_rand(10);X_rand(11) Y_rand(11);X_rand(12) Y_rand(12);X_rand(13) Y_rand(13);...
    X_rand(14) Y_rand(14);X_rand(15) Y_rand(15)];%���������(n+1)*2ά������[10 10]��Ŀ��λ�ã�ʣ�µĶ����ϰ���λ�á�(�����в����Ƴ������걣����һ�𣬵�һ��ΪĿ��㣬����ȫ���ϰ����)
Xj=Xo;%j=1ѭ����ʼ����������ʼ���긳��Xj
%***************��ʼ����������ʼ����ѭ��******************
for j=1:J %ѭ����ʼ
    Goal(j,1)=Xj(1); %Goal�Ǳ��泵�߹���ÿ��������ꡣ�տ�ʼ�Ƚ����Ž���������
    Goal(j,2)=Xj(2);
    
    %���ü���Ƕ�ģ��
    Theta=compute_angle(Xj,Xsum,n);%Theta�Ǽ�������ĳ����ϰ��Լ�Ŀ��֮�����X��֮��ļнǣ�ͳһ�涨�Ƕ�Ϊ��ʱ�뷽�������ģ����Լ��������

    %���ü�������ģ��
    Angle=Theta(1);%Theta��1���ǳ���Ŀ��֮��ĽǶȣ�Ŀ��Գ���������
    angle_at=Theta(1);%Ϊ�˺��������������������ķ�����ֵ��angle_at
    [Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle,b_goal,d_goal,n); %�����Ŀ��Գ���������x,y�������������ֵ��
    
    for i=1:n
        angle_re(i)=Theta(i+1);%��������õĽǶȣ��Ǹ���������Ϊ��n���ϰ�������n���Ƕȡ�
    end

    %���ü������ģ��
    [Frerxx,Freryy,Fataxx,Fatayy]=compute_repulsion(Xj,Xsum,m,angle_at,angle_re,n,Po,a);%�����������x,y����ķ������顣
    %��������ͷ����������⣬Ӧ��������ÿ��jѭ����ʱ������Ĵ�СӦ����һ��Ψһ�������������顣Ӧ�ðѳ��������з�����ӣ��������з�����ӡ�
    Fsumyj=Faty-Freryy+Fatayy;%y����ĺ���
    Fsumxj=Fatx-Frerxx+Fataxx;%x����ĺ���
    Position_angle(j)=atan(Fsumyj/Fsumxj);%������x�᷽��ļн�����

    % ���㳵����һ��λ��
    if (((Position_angle(j)>0.7)&&(Fsumyj<-0&&Fsumxj<-0))||(Fsumyj<0&&Fsumxj>0))%�ϰ����Ϸ�
        Xnext(1)=Xj(1)+l*cos(Position_angle(j)-pi/4);
        Xnext(2)=Xj(2)+l*sin(Position_angle(j)-pi/4);
    elseif ((Position_angle(j)<0.7)&&(Fsumyj<-0&&Fsumxj<-0))||(Fsumyj<0&&Fsumxj>0)%�ϰ����·�
        Xnext(1)=Xj(1)+l*cos(Position_angle(j)+pi/4);
        Xnext(2)=Xj(2)+l*sin(Position_angle(j)+pi/4);  
    else    
        Xnext(1)=Xj(1)+l*cos(Position_angle(j));
        Xnext(2)=Xj(2)+l*sin(Position_angle(j));
    end
    %���泵��ÿһ��λ����������
    Xj=Xnext;
    %�ж�
    if ((Xj(1)-Xsum(1,1))>0)&&((Xj(2)-Xsum(1,2))>0) %��Ӧ����ȫ��ȵ�ʱ�������������ֻ�ǽӽ��Ϳ���
        K=j;%��¼���������ٴΣ�����Ŀ�ꡣ
    break;
    %��¼��ʱ��jֵ
    end%���������if�����������·���ѭ��������ִ�С�
end%��ѭ������

K=j;
Goal(K,1)=Xsum(1,1);%��·�����������һ���㸳ֵΪĿ��
Goal(K,2)=Xsum(1,2);

%***********************************�����ϰ�����㣬Ŀ�꣬·����*************************
%����·��
X=Goal(:,1);
Y=Goal(:,2);
%·������Goal�Ƕ�ά����,X,Y�ֱ��������x,yԪ�صļ��ϣ�������һά���顣
x=X_rand;%�ϰ���xd����
y=Y_rand;

G_size = size(Goal,1);
fmat=moviein(G_size);
for j=1:G_size
    plot(100,100,'v',0,0,'ms',X(1:j),Y(1:j),'.r');
    axis([-5,105,-5,105]);
    set(gca,'xtick',(-5:1:105))
    set(gca,'ytick',(-5:1:105))
    grid on 
    hold on
    for k=1:2:15
        fill([x(k)-1 x(k)-1 x(k)+1 x(k)+1],[y(k)-1 y(k)+1 y(k)+1 y(k)-1],'k');
    end
    for k=2:2:14
        fill([x(k)-2 x(k)-2 x(k)+2 x(k)+2],[y(k)-2 y(k)+2 y(k)+2 y(k)-2],'k');
    end
    fmat(:,j)=getframe;
end
movie(fmat,10)