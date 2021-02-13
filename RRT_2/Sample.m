function X_rand=Sample(map,goal)
% if rand<0.5
%     X_rand = rand(1,2) .* size(map);   % random sample
% else 
%     X_rand=goal;
% end

if unifrnd(0,1)<0.5
   X_rand(1)= unifrnd(0,1)* size(map,1);   % ¾ùÔÈ²ÉÑù
   X_rand(2)= unifrnd(0,1)* size(map,2);   % random sample
else
   X_rand=goal;
end

