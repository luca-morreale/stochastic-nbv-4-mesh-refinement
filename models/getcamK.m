function K = getcamK(image_width, image_height, cam_right, cam_up, cam_dir, angle)
%%%
% Estimates the K matrix given the parameters used in the configuration of
% the camera in the pov file.
% Please specify the angle, both in the model and here, because gives
% better results.
%%%

% building getcamK(1920, 1080, [16/9 0 0], [0 1 0], [0 0 1], 1.5708)
% car      getcamK(1920, 1080, [16/9 0 0], [0 1 0], [0 0 1.5], 1.0472)
% fortress getcamK(1920, 1080, [16/9 0 0], [0 1 0], [0 0 1], 1.5708)
% tree     getcamK(1920, 1080, [4/3 0 0], [0 1 0], [0 0 1], 1.5708)


focal = 1;
aspect = norm(cam_right) / norm(cam_up);
if angle <=0
    angle = 2 * atan(norm(cam_right) / 2 / norm(cam_dir))
end

% pixel size 
psx = 2 * focal * tan(0.5 * angle) / image_width;
psy = 2 * focal * tan(0.5 * angle) / aspect / image_height;

psx = psx / focal; 
psy = psy / focal;

Ox = round(image_width * 0.5);
Oy = round(image_height * 0.5);


K = [1/psx     0     Ox;
            0    1/psy    Oy;
            0      0     1];
        
K(2,2) = -K(2,2);        

end
