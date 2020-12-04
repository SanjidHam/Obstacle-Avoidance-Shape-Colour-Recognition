clear;
clc;
rosshutdown
ipaddress = "192.168.43.10"
rosinit(ipaddress)
tbot = turtlebot('192.168.43.10')

laserSub = rossubscriber('/scan');
[velPub, velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
imsub = rossubscriber('/raspicam_node/image/compressed');

red=0;
green=0;
blueSquare=0;

flag=0;

    velMsg.Linear.X = 0;
	velMsg.Angular.Z = 0;
	velPub.send(velMsg);
while flag==0
    [red,green,blueSquare] = colourSegmentation();
    if  blueSquare==0
        turnNinety();
        disp("1")
    else          
            moveToObject();
            disp("2")
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = 0;
            velPub.send(velMsg);
            flag = 1;
            disp("3")
    end      
end
obstacleAvoidance(); 
disp("4")
%% FUNCTION: 90degree turn
function [] = turnNinety()
[velPub, velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
rate = rateControl(10);

while rate.TotalElapsedTime < 3
    velMsg.Linear.X = 0;
	velMsg.Angular.Z = 0.2;
	velPub.send(velMsg);
end
    velMsg.Linear.X = 0;
	velMsg.Angular.Z = 0;
	velPub.send(velMsg);
end    
%% FUNCTION: obstacleAvoidance
function [] = obstacleAvoidance()
    [velPub, velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
    laserSub = rossubscriber('/scan');
    
    vfh = controllerVFH;
    vfh.UseLidarScan = true;
    vfh.DistanceLimits = [0.05 1];
    vfh.RobotRadius = 0.105;
    vfh.MinTurningRadius = 0.2;
    vfh.SafetyDistance = 0.1;

    targetDir = 0;
rate = rateControl(10);

while rate.TotalElapsedTime < 10
    % Get laser scan data
    laserScan = receive(laserSub);
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);

    % Create a lidarScan object from the ranges and angles
    scan = lidarScan(ranges,angles);

    % Call VFH object to computer steering direction
    steerDir = vfh(scan, targetDir);  

    % Calculate velocities
    if ~isnan(steerDir) % If steering direction is valid
        desiredV = 0.2;
        w = exampleHelperComputeAngularVelocity(steerDir, 1);
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end

    % Assign and send velocity commands
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    velPub.send(velMsg);
    
%     set(figure,'Position',[50 50 800 400])
%     show(vfh)
end
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
velPub.send(velMsg); 
end
%% FUNCTION: moveToObject
function [] = moveToObject()
    [velPub, velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
    laserSub = rossubscriber('/scan');
    
    vfh = controllerVFH;
    vfh.UseLidarScan = true;
    vfh.DistanceLimits = [0.05 1];
    vfh.RobotRadius = 0.105;
    vfh.MinTurningRadius = 0.2;
    vfh.SafetyDistance = 0.1;

    % Get laser scan data
    laserScan = receive(laserSub);
    ranges = double(laserScan.Ranges);
    
    if ranges > 0.15
        desiredV = 0.15;
        w = 0;
    else
        desiredV = 0;
        w = 0;
    end

    % Assign and send velocity commands
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    velPub.send(velMsg);
end
%% FUNCTION: colourSegmentation
function [x,y,z] = colourSegmentation()
imsub = rossubscriber('/raspicam_node/image/compressed');
image = receive(imsub);
image.Format = 'bgr8; jpeg compressed bgr8';
figure(4);
img = readImage(image);
imshow(img);
%   d=impixel(img);

redBand = img(:,:, 1);
greenBand = img(:,:, 2);
blueBand = img(:,:, 3);

figure(1);
subplot (2,2,1);
imshow(img);
subplot (2,2,2);
imshow(redBand);
subplot (2,2,3);
imshow(greenBand);
subplot (2,2,4);
imshow(blueBand);

% thresholding all colours
redThreshold = 100;
greenThreshold = 70;
blueThreshold = 20;

redMask = (redBand > redThreshold) & (redBand < 160) & (greenBand > 50) &(greenBand < 110) & (blueBand > 45) & (blueBand<90);
greenMask = (greenBand > greenThreshold) & (greenBand < 110) & (redBand > 25) & (redBand < 40) & (blueBand > 35) & (blueBand<65) ;
blueMask = (blueBand > blueThreshold) & (blueBand < 90) & (redBand > 0) & (redBand < 40) & (greenBand > 25) &(greenBand < 60);

redMask = bwareaopen(redMask,500);
greenMask = bwareaopen(greenMask,500);
blueMask = bwareaopen(blueMask,50);

%BWR= imbinarize(redBand);
figure(2);
subplot(2,2,1);
imshow(redMask);
subplot(2,2,2);
imshow(greenMask);
subplot(2,2,3);
imshow(blueMask);
final= double(redMask)+double(greenMask)+double(blueMask);
subplot(2,2,4);
imshow(final);

%
final2=imfill(final,'holes');
final3=bwmorph(final2,'erode');
final4=bwmorph(final2,'dilate',1);

figure(3);
subplot(2,2,1);
imshow(final2);
subplot(2,2,2);
imshow(final3);
subplot(2,2,3);
imshow(final4);
% deducing the appropriate colour present
x=0;
y=0;
z=0;
siz=10;

r=regionprops(redMask,'Area');
g=regionprops(greenMask,'Area');
b=regionprops(blueMask,'Area');

R=isempty(regionprops(redMask,'Area'));
G=isempty(regionprops(greenMask,'Area'));
B=isempty(regionprops(blueMask,'Area'));
% nanmax(R)
if R==0 & max([r.Area])>siz
    x=1;
end
if G==0 & max([g.Area])>siz
    y=1;
end
if B==0 & max([b.Area])>siz
    FilledImage=imfill(blueMask,'holes');
    [labeledImage numberOfObjcts] = bwlabel(FilledImage);
    f = regionprops(labeledImage,'Perimeter','Area','FilledArea','Solidity','Centroid');

    per = [f.Perimeter];
    are = [f.Area];
    fil = [f.FilledArea];
    sol = [f.Solidity];

    cir = per .^2 ./ (4*pi*fil);

    for blobNumber = 1 : numberOfObjcts
      if cir(blobNumber) < 1.19
        shape = 'circle';
      else
        shape = 'square';
        z = 1;
      end
%       uiwait(msgbox(message));
    end
%         z=1;
end


end