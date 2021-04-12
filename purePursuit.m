function [angle,error,g] = purePursuit(q,L,Ld,Path)
%angle = calculated steering angle
%error = distance between robot position and closest point on path
%q(1) = x
%q(2) = y
%q(3) = heading
%q(4) = longitudinal velocity
%q(5) = steering angle
%L = vehicle wheelbase
%Ld = lookahead distance
%Path = desired path array

% Step 1: Find Closest Point on Path to Vehicle

seperation = sqrt((Path(1,:)-q(1)).^2+(Path(2,:)-q(2)).^2); %Minimum Distance Between Vehicle Position and Path Points
[~,i] = min(seperation); %i = index of smallest value of seperation

% Step 2: Identify Goal Point, Point on forwardPath closest to look ahead
% distance

[~,j] = min(abs(seperation(i:end) - Ld));

%Step 3: Transform Goal Point to Vehicle Coordinate System

%VTG = inv(); %Transformation matrix from Global Coordinates to vehicle coordinates
goalPoint = [cos(q(3)) -sin(q(3)) q(1);sin(q(3)) cos(q(3)) q(2); 0 0 1]\[Path(:,i+j-1);1]; %This has form (ex;ey)
errorPoint = [cos(q(3)) -sin(q(3)) q(1);sin(q(3)) cos(q(3)) q(2); 0 0 1]\[Path(:,i);1]; %This has form (ex;ey)

%Step 4: Calculate and Set New Steering Angle
angle = atan(2*goalPoint(2)*L/Ld);
error = sign(errorPoint(2))*sqrt(errorPoint(1)^2+errorPoint(2)^2); %+/- error is defined relative to vehicle y-axis
g = i+j-1; %Return goal point index
end