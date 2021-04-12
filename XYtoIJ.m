function [ i, j ] = XYtoIJ(x, y, Xmax, Ymax, R, C)
% compute pixel map coordinates from World cartesian coordinates
%   Detailed explanation goes here
%   -------->x
%  |
%  |
%  |
%y v
% i=1 is bottom row (y=Ymax)
% j=1 is left column (x=0)
i = round(((Ymax- y)/Ymax)*(R-1))+1;
j = round((x/Xmax)*(C-1))+1;
end

