function feasible=collisionChecking(startPose,goalPose,map)%��ͻ��飺�ж���ʼ�㵽�յ�֮���Ƿ����ϰ���

feasible=true;%���еģ���ִ�е�
dir=atan2(goalPose(1)-startPose(1),goalPose(2)-startPose(2));%Ŀ������ʼ��֮��ĽǶ�
for r=0:0.5:sqrt(sum((startPose-goalPose).^2))%sqrt(sum((startPose-goalPose).^2)):�����ľ���
    posCheck = startPose + r.*[sin(dir) cos(dir)];%��0.5�ļ���õ��м�ĵ�
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint([floor(goalPose(1)),ceil(goalPose(2))],map), feasible=false; end

end

function feasible=feasiblePoint(point,map)%�жϵ��Ƿ��ڵ�ͼ�ڣ���û���ϰ���
feasible=true;
if ~(point(1)>=1 &&  point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(2),point(1))==255)%map(point(2),point(1))==255��û���ϰ���
    feasible=false;
end

% function distance=Distance(start_Pt,goal_Pt)
% distance=sqrt((start_Pt(1)-goal_Pt(1))^2+(start_Pt(2)-goal_Pt(2))^2);
% 
% function [X_near,index]=Near(X_rand,T)
% min_distance=sqrt((X_rand(1)-T.v(1).x)^2+(X_rand(2)-T.v(1).y)^2);
% for T_iter=1:size(T.v,2)
%     temp_distance=sqrt((X_rand(1)-T.v(T_iter).x)^2+(X_rand(2)-T.v(T_iter).y)^2);
%     if temp_distance<=min_distance
%         min_distance=temp_distance;
%         X_near(1)=T.v(T_iter).x
%         X_near(2)=T.v(T_iter).y
%         index=T_iter;
%     end
% end
% 
% function X_rand=Sample(map,goal)
% % if rand<0.5
% %     X_rand = rand(1,2) .* size(map);   % random sample
% % else 
% %     X_rand=goal;
% % end
% 
% if unifrnd(0,1)<0.5
%    X_rand(1)= unifrnd(0,1)* size(map,1);   % ���Ȳ���
%    X_rand(2)= unifrnd(0,1)* size(map,2);   % random sample
% else
%    X_rand=goal;
% end
% 
% function X_new=Steer(X_rand,X_near,StepSize)
% theta = atan2(X_rand(1)-X_near(1),X_rand(2)-X_near(2));  % direction to extend sample to produce new node
% X_new = X_near(1:2) + StepSize * [sin(theta)  cos(theta)];
% 
% % dir_x = X_rand(1)- X_near(1);
% % dir_y = X_rand(2)- X_near(2);
% % dir = sqrt(dir_x^2 + dir_y^2);
% % X_new(1) = dir_x * StepSize/dir+X_near(1);
% % X_new(2) = dir_y * StepSize/dir+X_near(2);
% 
% function X_rand=Sample(map,goal)
% % if rand<0.5
% %     X_rand = rand(1,2) .* size(map);   % random sample
% % else 
% %     X_rand=goal;
% % end
% 
% if unifrnd(0,1)<0.5
%    X_rand(1)= unifrnd(0,1)* size(map,1);   % ���Ȳ���
%    X_rand(2)= unifrnd(0,1)* size(map,2);   % random sample
% else
%    X_rand=goal;
% end