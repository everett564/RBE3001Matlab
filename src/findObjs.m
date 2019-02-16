% findObjs detects objects that may be present in an RGB image
function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)

% image enhancement
 
    % Undistort the image
    [im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
    
    % Mask the Images
    [mask,im2] = createMask7(im);
    [mask,im3] = createMask8(im);
    
   
%% Color
 im2 = imfilter(im2,[1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9]);
 im2 = rgb2gray(im2);

figure; imshow(im2, 'InitialMagnification', 100);
title('Segmented Dots');

[centers, radii, metric] = imfindcircles(im2,[20 60],'Sensitivity', .90);

viscircles(centers, radii,'EdgeColor','b');

hold on
plot(centers(:,1),centers(:,2), 'b*')
hold off

newCent = [(centers(:,1)- radii) (centers(:,2)-radii)];
location = [newCent radii*2 radii*2];
colors = cell(size(location,1),1);

for i = 1:size(centers,1)
    
    row = centers(i,:);
    pixVal = impixel(im,row(1),row(2));
    for j= 1:radii/5
    pixVal = [pixVal;impixel(im,row(1)+j,row(2)+j)];
    end
    pixVal = mean(pixVal);
    
    if pixVal(1) > 70 && pixVal(3) > 100 
        colors{i} = ['green:' num2str(pixVal)];
    elseif pixVal(3) > 190  
         colors{i} =  ['blue:' num2str(pixVal)];
    else 
         colors{i} =  ['yellow:' num2str(pixVal)];
    end
    
end



%% Black and White
 
 im3 = imfilter(im3,[1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9]);
 im3 = rgb2gray(im3);
 
 [~, threshold] = edge(im3, 'sobel');
 edge(im3,'sobel', threshold);
 im3 = imfill(im3);
    
    
figure; imshow(im3, 'InitialMagnification', 100);
title('Segmented Bases');
    
[centers, radii, metric] = imfindcircles(im3,[30 70], 'Sensitivity', .94);

viscircles(centers, radii,'EdgeColor','b');

hold on
plot(centers(:,1),centers(:,2), 'b*')
hold off

% Image Analysis


% Insert labels for the coins.
im = insertObjectAnnotation(im, 'rectangle', location, colors);
figure; imshow(im);
title('Detected');

%%

 %diskDia = sqrt((4*area)/pi);
 
% convert camera coordinates to checkerboard reference frame
 %robotFramePose = checkerboardTransformation(centroid);
 
 %imDetectedDisk = insertShape(imOrig,'FilledCircle',[centroid(1),centroid(2),5]);
 

end