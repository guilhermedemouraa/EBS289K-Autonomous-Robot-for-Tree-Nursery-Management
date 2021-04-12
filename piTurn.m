function [j,pathX,pathY] = piTurn(point1,point2,dx,rMin)
% Function generates path points dx units apart from point1 to point2 for a
% pi turn given the vehicles minimum turning radius
% Point Format: [x;y] 2 x N matrix
% Function assumes bottom of field is coincident with x axis
%Construct Generic Path Between Two Points
%Generic Path is top of field right turn
a = pi:-dx:pi/2;
b = pi/2:-dx:0;
if point1(1)<point2(1)
    pointA = point1;
    pointB = point2;
    genPath = [(pointA(1)+rMin+rMin*cos(a)), ((pointA(1)+rMin):dx:(pointB(1)-rMin)),(pointB(1)-rMin+rMin*cos(b));pointA(2)+rMin*sin(a), (pointA(2)+rMin)*ones(1,length(((pointA(1)+rMin):dx:(pointB(1)-rMin)))),(pointA(2)+rMin*sin(b))];
else
    pointA = point2;
    pointB = point1;
    genPath = [(pointA(1)+rMin+rMin*cos(a)), ((pointA(1)+rMin):dx:(pointB(1)-rMin)),(pointB(1)-rMin+rMin*cos(b));pointA(2)+rMin*sin(a), (pointA(2)+rMin)*ones(1,length(((pointA(1)+rMin):dx:(pointB(1)-rMin)))),(pointA(2)+rMin*sin(b))];
    genPath = flip(genPath,2);
end

if point1(2) ~= 20 %Top of Row
        pathX = genPath(1,:);
        pathY = genPath(2,:);
else %Bottom of the field
    genPath(2,:) = -genPath(2,:); %Y points are reflected over x axis
        pathX = genPath(1,:);
        pathY = 40+genPath(2,:);

end
j = length(pathX);
end