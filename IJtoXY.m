function [ x, y ] = IJtoXY(i, j, Xmax, Ymax, R, C)
% compute World cartesian coordinates  from pixel map coordinates
y = Ymax - Ymax * (i-1)/(R-1);
x = Xmax *(j-1)/(C-1);

end

