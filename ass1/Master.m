% Autonome Mobiele Robots Opdr 1 Tim Bloeme & Wolf Vos
% For this assignment we had to implement motorcontrols for the LEGO Brick
% robot and let it drive over a small trajectory of turns and straight
% combined.

%this function is the Master function and this will function as a main
%function.
function  Master()
%opening a connection with the LEGO Brick
h = COM_OpenNXT();
COM_SetDefaultNXT(h);

Z = [0,0,pi/2]'; 

%These are the actions the robot will perform.
[Z] = Straight(20, Z)
[Z] = Turn(1, -15, Z)
[Z] = Straight(20, Z)
[Z] = Turn(1, 15, Z)
[Z] = Straight(20, Z)

%closing the connection
COM_CloseNXT('all');
end

%This function will allow the robot to go straight for a certain distance.
function [Z] = Straight(cm, Z)
Z
x = cm*360/17.854;
l = 11.5/2;
r = 5.6;
%check if the distance is positive of negative for going forward or
%backwards.
if cm > 0
    power = 25;
else
    power = -25;
end
%Sending commands to the motors.
    NXT_SetOutputState(MOTOR_B, power, true, true, 'SPEED', 0, 'RUNNING', abs(x), 'dontreply');
    NXT_SetOutputState(MOTOR_C, power, true, true, 'SPEED', 0, 'RUNNING', abs(x), 'dontreply');
    NXT_GetOutputState(MOTOR_B);
    NXT_GetOutputState(MOTOR_C);
    
    %Calculating the rotation and movement matrix so that we can translate
    %it to the global reference plane. 
    R = rotMatrix(Z(3));
    R = round(R);
    Er = RMoveVector(abs(x)/360.0, abs(x)/360.0, r, l)
    Z = Z + (R * Er);
    pause(2);
end

%This function will allow the robot to turn left of right over a circle
%with radius 'draaicirkel'. It will do the amount of radians specified in
%the 'rad' variable.
function [Z] = Turn(rad, draaicirkel, Z)
    l = 11.5/2;
    r = 5.6;
    power = 25;
    
    %Calculate the distance each wheel has to drive in order to make the
    %turn.
    distB = rad*pi*(abs(draaicirkel-l));
    distC = rad*pi*(abs(draaicirkel+l));
    rotB = distB*360/17.854;
    rotC = distC*360/17.854;
    %calculate the relation between the two distances because the power of
    %the wheels need to have the same relation.
    if distB < distC
        factor = distB/distC;
    else 
        factor = distC/distB;
    end
if draaicirkel < 0
    powerB = power;
    powerC = power*factor;
else
    powerB = power*factor;
    powerC = power;
end
if rad < 0
    powerB = -powerB;
    powerC = -powerC;
end
%Sending commands to the motors.
    NXT_SetOutputState(MOTOR_B, powerB, true, true, 'SPEED', 0, 'RUNNING', abs(rotB), 'dontreply');
    NXT_SetOutputState(MOTOR_C, powerC, true, true, 'SPEED', 0, 'RUNNING', abs(rotC), 'dontreply');
    NXT_GetOutputState(MOTOR_B);
    NXT_GetOutputState(MOTOR_C);
    
    %Calculating the rotation and movement matrix so that we can translate
    %it to the global reference plane. 
    R = rotMatrix(Z(3));
    R = round(R);
    Er = RMoveVector(abs(rotB)/360, abs(rotC)/360, r, l)
    
    Z = Z + (R * Er);
    pause(2);
end

%This fucntion will compute the rotation matrix.
function [R] = rotMatrix(theta)
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0;0,0,1];
end
%This function will compute the movement matrix in the robots reference
%plane.
function [Er] = RMoveVector(powerB, powerC, r, l)
    Er = [((r*powerB)/2 )+( (r*powerC)/2) , 0, ((r*powerB)/(2*l))+((-r*powerC)/(2*l))]'; 
end