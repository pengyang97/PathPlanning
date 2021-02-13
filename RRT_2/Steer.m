function X_new=Steer(X_rand,X_near,StepSize)
theta = atan2(X_rand(1)-X_near(1),X_rand(2)-X_near(2));  % direction to extend sample to produce new node
X_new = X_near(1:2) + StepSize * [sin(theta)  cos(theta)];

% dir_x = X_rand(1)- X_near(1);
% dir_y = X_rand(2)- X_near(2);
% dir = sqrt(dir_x^2 + dir_y^2);
% X_new(1) = dir_x * StepSize/dir+X_near(1);
% X_new(2) = dir_y * StepSize/dir+X_near(2);

