% findObjs detects objects that may be present in an RGB image
function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig, cameraParams)

% image enhancement
 [im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
 im = medfilt2(im);
 %im = histeq(im);

% segmentation
 [~, threshold] = edge(im, 'sobel');
 fudgeFactor = .5;
 lines = edge(im,'sobel', threshold * fudgeFactor);
 
% post-processing
 se90 = strel('line', 4, 90);
 se0 = strel('line', 4, 0);
 dilate = imdilate(lines, [se90 se0]);
 border = imclearborder(dilate, 4);
 
 bw = imfill(border, 'holes');
 
 seD = strel('diamond',4);
 smooth = imerode(bw,seD);
 smooth = imerode(smooth,seD);
 
% information extraction
 info = regionprops(smooth, 'Area', 'Centroid');
 centroid = info.Centroid;
 area = info.Area;

 diskDia = sqrt((4*area)/pi);
 
% convert camera coordinates to checkerboard reference frame
 robotFramePose = checkerboardTransformation(centroid);
 
 imDetectedDisk = insertShape(imOrig,'FilledCircle',[centroid(1),centroid(2),5]);

end