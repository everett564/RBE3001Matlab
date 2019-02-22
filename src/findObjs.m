% findObjs detects objects that may be present in an RGB image
function [imOutput, robotFramePose, diskDia, colors] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)

% image enhancement
 
    % Undistort the image
    [im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
    
    % Mask the Images
    [mask,im2] = createMask7(im);
    [mask,im3] = createMask4(im);
    
  
%% Color
 
im2 = imfilter(im2,[1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9]);
 im2 = rgb2gray(im2);

%figure; imshow(im2, 'InitialMagnification', 100);
%title('Segmented Dots');

[centers, radii, metric] = imfindcircles(im2,[20 60],'Sensitivity', .90);

if size(centers) > 0
viscircles(centers, radii,'EdgeColor','b');

hold on
 
plot(centers(:,1),centers(:,2), 'b*')

hold off

newCent = [(centers(:,1)- radii) (centers(:,2)-radii)];
location = [newCent radii*2 radii*2];
colors = cell(size(location,1),1);

for i = 1:size(centers,1)
    s = (T_cam_to_checker);
    R = s(1:3,1:3);
    t = s(1:3,4);
    worldPoints(i,:) = pointsToWorld(cameraParams, R, t, centers(i,:));
    worldPoints(i,1) = -1*( worldPoints(i,1) - 275.8);
    worldPoints(i,2) =  worldPoints(i,2) + 113.6;
    
    

    row = centers(i,:);
    pixVal = impixel(im,row(1),row(2));
    for j= 1:radii/5
    pixVal = [pixVal;impixel(im,row(1)+j,row(2)+j)];
    end
    
    
    pixVal = mean(pixVal);
    
    if pixVal(1) > 70 && pixVal(3) > 100 
        colors{i} = ['green:' num2str(worldPoints(i,:))];
    elseif pixVal(3) > 150  
         colors{i} =  ['blue:' num2str(worldPoints(i,:))];
    else 
         colors{i} =  ['yellow:' num2str(worldPoints(i,:))];
    end
    
end
im5 = insertObjectAnnotation(im, 'rectangle', location, colors);

end

%% Bases
 
 im3 = imfilter(im3,[1/7 1/7 1/7; 1/7 1/7 1/7; 1/7 1/7 1/7]);
 im3 = rgb2gray(im3);
 
 [~, threshold] = edge(im3, 'sobel');
 edge(im3,'sobel', threshold);
 im3 = imfill(im3);
    
    
%figure; imshow(im3, 'InitialMagnification', 100);
%title('Segmented Bases');
    
[centers2, radii2, metric] = imfindcircles(im3,[30 70], 'Sensitivity', .94);

if size(centers2) > 0 
viscircles(centers2, radii2,'EdgeColor','b');

hold on
robotFramePose = [];
diskDia = [];
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


% Insert labelsim5 = insertObjectAnnotation(im, 'rectangle', location, colors);


%figure; imshow(im5);
%title('Detected Colors');


%im4= insertObjectAnnotation(im5, 'rectangle', locations, sizes);



%figure; imshow(im4);
%title('Detected Bases');
image(im5);
imOutput = im5;

robotFramePose = worldPoints;
diskDia = sizes;


else
image(im);
imOutput = im;
robotFramePose = [];
diskDia = [];

end


end