ptCloud = pcread('/Users/holly/Desktop/1495477573738894raw.pcd')

%% Rotate point clouds
theta = 0;
R1 = [1, 0, 0, 0; 
     0, cos(theta), -sin(theta), 0; 
     0, sin(theta), cos(theta), 0; 
     0, 0, 0, 1];

beta = -0.24;
R2 = [cos(beta),  0,  sin(beta), 0;
      0,          1,          0, 0;
      -sin(beta), 0,  cos(beta), 0;
      0,          0,          0, 1]

gamma = -0.4;
R3 = [cos(gamma),   -sin(gamma),  0, 0;
      sin(gamma),    cos(gamma),  0, 0;
               0,             0,  1, 0;
               0,             0,  0, 1]

  
t = [0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];
  
R = R1 * R2 * R3 + t



trans = affine3d(R);
ptCloudOut = pctransform(ptCloud,trans);

pcshow(ptCloudOut)
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Creat 2.5D Grid Map
grid_size = 0.2;
grid_size_z = 0.05;
x_min = ptCloudOut.XLimits(1);
y_min = ptCloudOut.YLimits(1);
z_min = ptCloudOut.ZLimits(1);

map_x = ceil((ptCloudOut.XLimits(2) - ptCloudOut.XLimits(1))/grid_size);
map_y = ceil((ptCloudOut.YLimits(2) - ptCloudOut.YLimits(1))/grid_size);
map_z = ceil((ptCloudOut.ZLimits(2) - ptCloudOut.ZLimits(1))/grid_size_z);

map = zeros(map_x+1, map_y+1, map_z+1);

i = 0;
for h = 1:1920
    for w = 1:1080
        if ~isnan(ptCloudOut.Location(h, w, 1)) && ~isnan(ptCloudOut.Location(h, w, 2)) && ~isnan(ptCloudOut.Location(h, w, 3))

             x_idx = ceil((ptCloudOut.Location(h, w, 1) - x_min)/grid_size)+1;
             y_idx = ceil((ptCloudOut.Location(h, w, 2) - y_min)/grid_size)+1;
             z_idx = ceil((ptCloudOut.Location(h, w, 3) - z_min)/grid_size_z)+1;
             map(x_idx, y_idx, z_idx) = map(x_idx, y_idx, z_idx) + 1;
        end
        i = i + 1;
        if  mod(i,10000) == 0
            disp(i)
        end
        
    end
end


% 
% for idx = 1:ptCloudOut.Count
%     pt = ptCloudOut.select(idx);
%     if ~isnan(pt.Location(1)) && ~isnan(pt.Location(2)) && ~isnan(pt.Location(3))
% 
%          x_idx = ceil((pt.Location(1) - x_min)/grid_size);
%          y_idx = ceil((pt.Location(2) - y_min)/grid_size);
%          z_idx = ceil((pt.Location(3) - z_min)/grid_size_z);
% 
%          map(x_idx, y_idx, z_idx) = map(x_idx, y_idx, z_idx) + 1;
%     end
%     
%    
% end

%% collision value
car_height_min = - 0.9;
car_height_max = car_height_min + 0.5;

car_min_idx = ceil((car_height_min - z_min)/grid_size_z)+1;
car_max_idx = ceil((car_height_max - z_min)/grid_size_z)+1;

collision_map = sum(map(:,:, car_min_idx:car_max_idx), 3);
collision_map(collision_map>1000) = 1000;
% collision_map(collision_map<250) = 0;

collision_map = collision_map./max(max(collision_map));
b = bar3(collision_map);
colorbar
















