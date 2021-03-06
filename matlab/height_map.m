ptCloud = pcread('./data/example.pcd');
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

%%
heightmap = ptCloudOut.Location(:, :, 3);
heightmap(isnan(heightmap)) = min(min(heightmap));

heightmap = heightmap - min(min(heightmap));
heightmap = heightmap / max(max(heightmap));
heightmap = im2uint8(heightmap);
heighrCImage = ind2rgb(heightmap, jet(256));
heighrCImage = im2uint8(heighrCImage);

imshow(heighrCImage);

combined_image = heighrCImage * 0.7 + colormap * 0.3;
imshow(combined_image)
