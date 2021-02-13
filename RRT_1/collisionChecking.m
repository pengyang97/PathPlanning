function feasible=collisionChecking(startPose,goalPose,map)     %��ͻ��飺�ж���ʼ�㵽�յ�֮���Ƿ����ϰ���

feasible=true;      %���еģ���ִ�е�
dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));   %Ŀ������ʼ��֮��ĽǶ�
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))  %�����ľ���
    posCheck = startPose + r.*[sin(dir) cos(dir)];%��0.5�ļ���õ��м�ĵ�
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end

end

function feasible=feasiblePoint(point,map)%�жϵ��Ƿ��ڵ�ͼ�ڣ���û���ϰ���
feasible=true;
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)%û���ϰ���
    feasible=false;
end


