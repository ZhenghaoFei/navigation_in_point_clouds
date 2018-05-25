ptCloud = pcread('./data/example.pcd');

player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits);
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

xlabel('X');
ylabel('Y');
zlabel('Z');
%%
maxDistance = 0.2; % in meters
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(ptCloudOut, maxDistance, referenceVector);

%%
blue = [0, 0, 1];
red = [1, 0, 0];
green = [0, 1, 1];


% Select the points that are not part of the ground plane.
pcWithoutGround = select(ptCloudOut, inPlanePointIndices);
obs_pc = select(ptCloudOut, outliers);

pcshow(obs_pc.Location, red);
hold
pcshow(pcWithoutGround.Location, green);



