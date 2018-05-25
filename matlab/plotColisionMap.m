grid_size = 0.2;
ground_height = -1.2;
map_size = size(collision_map);
h = map_size(1);
w = map_size(2);


for h_idx = 0:h-1
    for w_idx = 0:w-1
        x = [h_idx * grid_size, (h_idx+1) * grid_size, (h_idx+1) * grid_size h_idx *grid_size] + x_min - grid_size;
        y = [w_idx * grid_size, w_idx * grid_size, (w_idx+1) * grid_size, (w_idx+1) * grid_size] + y_min -grid_size;
        z = [ground_height, ground_height, ground_height, ground_height];
        if collision_map(h_idx+1, w_idx+1) > 0
            p = patch(x, y, z, 'red');

        else
            p = patch(x, y, z, 'green');
            a = 1;
        end
        p.FaceAlpha = 0.1;
    end
end