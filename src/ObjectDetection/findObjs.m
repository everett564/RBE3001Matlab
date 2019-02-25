% Find Objects Function
% findObjs detects objects that may be present in an RGB image
% imOrig - the original image from the camera
% T_checker_to_robot - the checker to robot transformation matrix
% T_cam_to_checker - the camera to checker transformation matrix
% Camera Params - Camera Parameters
% Returns
% imOutput - the image output with tags on the bases and balls
% robotFramePose - the pose the robot needs to be at to grab the balls
% colorAndBase - a matrix with all of the colors and bases of the objects
% in the space
% colorsOut - a matrix of all of the colors

function [imOutput, robotFramePose, colorAndBase, colorsOut] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)

%% Image enhancement

% Undistort the image
[im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

% Mask the Images
% Mask for color
[mask,im2] = createMask7(im);
% Mask for Bases
[mask,im3] = createMask4(im);

%Intialize the Matricies Used Later
colorAndBase = [];
colorsOut = [];
location = [];
colors = [];

%% Color

% Filters
im2 = imfilter(im2,[1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9]);
im2 = rgb2gray(im2);

% Find the Balls
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
        worldPoints(i,1) = -1*( worldPoints(i,1) - 285.8); %275.8
        worldPoints(i,2) =  worldPoints(i,2) + 113.6; %113.6
        
        
        row = centers(i,:);
        pixVal = impixel(im,row(1),row(2));
        for j= 1:radii/5
            pixVal = [pixVal;impixel(im,row(1)+j,row(2)+j)];
        end
        
        
        pixVal = mean(pixVal);
        
        if pixVal(3) > 150 && pixVal(3) > pixVal(2) && pixVal(3) > pixVal(1)
            colors{i} =  ['blue:' num2str(pixVal)];
            colorsOut(i) = 2;
        elseif (pixVal(2) - pixVal(1)) > 50 && (pixVal(2) - pixVal(1)) > 50
            colors{i} = ['green:' num2str(pixVal)];
            colorsOut(i) = 1;
        else
            colors{i} =  ['yellow:' num2str(pixVal)];
            colorsOut(i) = 3;
        end
        
    end
    im5 = insertObjectAnnotation(im, 'rectangle', location, colors);
    
end

%% Bases

% Filters
im3 = imfilter(im3,[1/7 1/7 1/7; 1/7 1/7 1/7; 1/7 1/7 1/7]);
im3 = rgb2gray(im3);

% Threshold the image
[~, threshold] = edge(im3, 'sobel');
edge(im3,'sobel', threshold);
im3 = imfill(im3);

% Find the Bases
[centers2, radii2, metric] = imfindcircles(im3,[40 70], 'Sensitivity', .955);

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
    
    % Print the Bases Sizes
    for i = 1:size(centers2,1)
        
        row = radii2(i,:);
        
        if row > 50
            sizes{i} = ['Large: ', num2str(row)];
            sizeOut(i) = 2;
        else
            sizes{i} = ['Small: ', num2str(row)];
            sizeOut(i) = 1;
        end
        
    end
    
    
    for k = 1:size(centers,1)
        sizeOfBase = findNearestBase(centers(k,:),centers2,sizeOut);
        if colorsOut(k) > 0
            colorAndBase(k,:) = [colorsOut(k) sizeOfBase];
        end
    end
    
    if (size(location,1) > 0) && (size(colors,1) > 0)
        im4= insertObjectAnnotation(im5, 'rectangle', locations, sizes);
        
        image(im4);
        imOutput = im5;
    else
        imOutput = im;
    end
    
    robotFramePose = worldPoints;
    diskDia = sizes;
    
else
    
    image(im);
    imOutput = im;
    robotFramePose = [];
    diskDia = [];
    
end


end