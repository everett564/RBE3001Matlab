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
    elseif pixVal(3) > 150  
         colors{i} =  ['blue:' num2str(pixVal)];
    else 
         colors{i} =  ['yellow:' num2str(pixVal)];
    end
    
end



%% Bases
 
 im3 = imfilter(im3,[1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9]);
 im3 = rgb2gray(im3);
 
 [~, threshold] = edge(im3, 'sobel');
 edge(im3,'sobel', threshold);
 im3 = imfill(im3);
    
    
figure; imshow(im3, 'InitialMagnification', 100);
title('Segmented Bases');
    
[centers2, radii2, metric] = imfindcircles(im3,[30 70], 'Sensitivity', .9425);

viscircles(centers2, radii2,'EdgeColor','b');

hold on
plot(centers2(:,1),centers2(:,2), 'b*')
hold off

newCent2 = [(centers2(:,1)- radii2) (centers2(:,2)-radii2)];
locations = [newCent2 radii2*2 radii2*2];
sizes = cell(size(locations,1),1);

for i = 1:size(centers2,1)
    
    row = radii2(i,:);
    
    if row > 50 
       sizes{i} = ['Large: ', num2str(row)];
    else 
       sizes{i} = ['Small: ', num2str(row)];
    end
    
end


% Insert labels
im5 = insertObjectAnnotation(im, 'rectangle', location, colors);
figure; imshow(im5);
title('Detected Colors');

%
im4 = insertObjectAnnotation(im, 'rectangle', locations, sizes);
figure; imshow(im4);
title('Detected Bases');
%%

 %diskDia = sqrt((4*area)/pi);
 
% convert camera coordinates to checkerboard reference frame
 %robotFramePose = checkerboardTransformation(centroid);
 
 %imDetectedDisk = insertShape(imOrig,'FilledCircle',[centroid(1),centroid(2),5]);
 

end