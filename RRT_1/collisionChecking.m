function feasible=collisionChecking(startPose,goalPose,map)     %冲突检查：判断起始点到终点之间是否有障碍物

feasible=true;      %可行的，可执行的
dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));   %目标点和起始点之间的角度
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))  %两点间的距离
    posCheck = startPose + r.*[sin(dir) cos(dir)];%以0.5的间隔得到中间的点
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end

end

function feasible=feasiblePoint(point,map)%判断点是否在地图内，且没有障碍物
feasible=true;
if ~(point(1)>=1 &&  point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==255)%没有障碍物
    feasible=false;
end


