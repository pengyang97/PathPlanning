% RRT* algorithm in 2D with collision avoidance.
% 
% Author: Sai Vemprala
% 
% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
% 
% Brief description of algorithm: 
% 1. Pick a random node q_rand.     ���ѡȡһ���ڵ�
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.    ��̽���Ľڵ㸽���ҵ�����Ľڵ�q_near�����з�֧��q_rand
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.     ��q_nearת��q_rand������ڵ�̫Զ�����ֵ����չ��������q_new,������ϰ����Ƿ񱻻��С�
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.   ���´�q_near����q_new�ĳɱ��������ΪCmin��Ŀǰ��q_near�䵱q_new�ĸ��ڵ㡣
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.    �ӡ��ѷ��ʡ��ڵ���б��У�����ڸ����뾶��Χ��������ھӣ������б�q_nearest�С�
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.   ��q_nearest������г�Ա�У�����Ƿ��дӲ�ͬ�ĸ��ڵ��Ե���Cmin�Ĵ��۵���q_new�����Ҳ������ϰ�����ײ�����У�ѡ��q_nearest�гɱ���͵Ľڵ㲢�������Ϊq_new�ĸ��ڵ㡣
% 7. Add q_new to node list. �������ɵĽڵ�ӽ��ڵ�����ȥ
% 8. Continue until maximum number of nodes is reached or goal is hit. ѭ����������ֱ���ڵ����������õ���Ŀ�����ҵ�Ŀ���

clearvars
close all

x_max = 1000;
y_max = 1000;
obstacle = [500,150,200,200];
EPS = 20;   %��չ����
numNodes = 3000;     %�����������ܸ������൱�����õĵ�������   

q_start.coord = [0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [999 999];
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)
axis([0 x_max 0 y_max])
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
hold on

for i = 1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached  ����ҵ�Ŀ���������ѭ��
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    % �ҳ���֪�ڵ��о��������������Ǹ���
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);    %����steer���������µĽڵ�
    %��δ���ע��һ�£���ײ�жϺ����еĲ�������һ��Ӧ����q_new.coord,��Ӧ����q_near.coord��
    if noCollision(q_new.coord, q_near.coord, obstacle)  %�ж������ɵ�������Ƿ�������ϰ����У���������ڣ��������ɵĽڵ�������Ľڵ��Ժ��߽�������
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        % �������ɵĽڵ�Ϊ���ģ���r=60Ϊ�뾶��Բ��Ȧ���½ڵ����п��ܵĸ��ڵ�
        q_nearest = [];
        r = 60;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower cost paths  �ҳ��½ڵ�Ǳ�ڵĸ��ڵ�
        
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end
