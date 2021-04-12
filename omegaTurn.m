function [j,pathX,pathY] = omegaTurn(point1,point2,rMin,dW)
% Function generates path points from point1 to point2 for an
% omega turn given the vehicles minimum turning radius
% Point Format: [x;y] 2 x N matrix
% Function assumes bottom of field is coincident with x axis

% Construct Generic Path Between Two Points
% Generic Path is top of field right turn
B = acos(1-((2*rMin+dW)^2)/(8*rMin^2));
A = (pi-B)/2;
a = 0:0.1:A;
b = pi+A:-0.1:-A;
c = pi-A:0.1:pi;

if point1(1)<point2(1)
    pointA = point1;
    pointB = point2;
    genPath = [pointA(1)-rMin+rMin*cos(a), pointA(1)+dW/2+rMin*cos(b),pointB(1)+rMin+rMin*cos(c);pointA(2)+rMin*sin(a),pointA(2)+2*rMin*sin(A)+rMin*sin(b),pointB(2)+rMin*sin(c)];
else
    pointA = point2;
    pointB = point1;
    genPath = [pointA(1)-rMin+rMin*cos(a), pointA(1)+dW/2+rMin*cos(b),pointB(1)+rMin+rMin*cos(c);pointA(2)+rMin*sin(a),pointA(2)+2*rMin*sin(A)+rMin*sin(b),pointB(2)+rMin*sin(c)];
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
