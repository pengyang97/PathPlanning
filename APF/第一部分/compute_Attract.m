%�Ľ������Ƴ����������ĺ���
function [Yatx,Yaty]=compute_Attract(X,Xsum,k,angle,b_goal,d_goal,n)%�������Ϊ������ǰ���꣬Ŀ�����꣬���泣��,���������ĽǶ�
    %��·���ϵ���ʱ����Ϊÿ��ʱ�̵�Xgoal
    R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%·�����Ŀ��ľ���ƽ��
    r=sqrt(R);%·�����Ŀ��ľ���

    if r>d_goal
        r=d_goal;
    elseif r<b_goal%��ֹ�����ھ���Ŀ��㸽��ʱ��
        k=k*20;
        r=b_goal;
    %r>=b_goal && r<=d_goal
    else
        k=k*5;    
    end

    Yatx=k*r*cos(angle);%angle=Y(1)
    Yaty=k*r*sin(angle);
end
