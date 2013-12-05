%LASERSCAN_TEST
%
%   Author Davide Scaramuzza - davide.scaramuzza@ieee.org
%   ETH Zurich - April, 25, 2007

%stop(vid);
% Configure the object for manual trigger mode.
%triggerconfig(vid, 'manual');

% Now that the device is configured for manual triggering, call START.
% This will cause the device to send data back to MATLAB, but will not log
% frames to memory at this point.
%start(vid)

% -------------------------------------------------------------------------
% MOST IMPORTANT PARAMETERS
% -------------------------------------------------------------------------
% Rmax = 160;%        Max detectable distance, set to 160 pixels in VGA images.
%                     Rmax was already loaded when calling "calibrate_camera.m"
% Rmin = 77;%         Min detectable distance in pixels in VGA image
%                     Rmin was already loaded when calling "calibrate_camera.m"
alpha = 150;%         Radial distortion coefficient, YOU MAY NEED TO TUNE THIS PARAMETER!!!!!!!!!!!!!!!!!!!!!!!!!!!!
height = 0.21;%       camera height in meters, YOU MAY NEED TO TUNE THIS PARAMETER!!!!!!!!!!!!!!!!!!!!!!!!!!!!
BWthreshold = 100;%    Threshold for segment the image into Black & white colors, YOU MAY NEED TO TUNE THIS PARAMETER!!!!!!!!!!!!!!!!!!!!!!!!!!!!
angstep = 1.0;%       Angular step of the beam in degrees
axislimit = 0.8;%     Axis limit
Rmin = 125;%           Overwrite value from calibration step.
Rmax = 205;
center = [457.7,342];
       

% -------------------------------------------------------------------------
% MAIN
% -------------------------------------------------------------------------
for i=1:1
    tic;%                               Start counting elapsed time
    
    %snapshot = getsnapshot(vid);%       Acquire image
    snapshot = imread('picture3.jpg');
    snapshot = imflipud(snapshot);
    
    %calibrate_camera_offline;  %we used this to get the points of the
                                %picture.
    
    
    [undistortedimg, theta] = imunwrap(snapshot, center, angstep, Rmax, Rmin);
    BWimg = img2bw(undistortedimg, BWthreshold);
    rho = getpixeldistance(BWimg, Rmin);
    figure(1); 
    imagesc( snapshot ); 
    hold on; 
    drawlaserbeam( center, theta, rho ); 
    hold off;    
    figure(2);
    dist = undistort_dist_points(theta, rho, alpha, height);
    draw_undistorted_beam(dist, theta, axislimit);
    ind = find(rho ~= inf);
    rho = rho(ind);
    
    sigma_dist = compute_uncertainty(rho, std(rho), alpha, height);
    hold on
    draw_uncertainty(dist(ind), theta(ind), sigma_dist)
    
    

    
    
% Compute the time per frame and effective frame rate.
elapsedTime = toc;
effectiveFrameRate = 1/elapsedTime
end