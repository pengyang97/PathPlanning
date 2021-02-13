function [X_near,index]=Near(X_rand,T)
min_distance=sqrt((X_rand(1)-T.v(1).x)^2+(X_rand(2)-T.v(1).y)^2);
for T_iter=1:size(T.v,2)
    temp_distance=sqrt((X_rand(1)-T.v(T_iter).x)^2+(X_rand(2)-T.v(T_iter).y)^2);
    if temp_distance<=min_distance
        min_distance=temp_distance;
        X_near(1)=T.v(T_iter).x
        X_near(2)=T.v(T_iter).y
        index=T_iter;
    end
end

