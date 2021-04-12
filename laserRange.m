%Uses the Bresenham algorithm to generate pixels along a line (x1, y1, x2, y2) and 
%returns the distance of (x1, y1) to the first occupied pixel in a map;
%occupied pixels ahve value of 1; otherwise, 0. No obstacles returns inf.
% Modified code by S. Vougioukas
% Copyright (C) 1993-2014, by Peter I. Corke
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function p = laserRange(p1, p2, map)
   [R, C] = size(map);
   
    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);
    

    x = x1;
    if x2 > x1
        xd = x2-x1;
        dx = 1;
    else
        xd = x1-x2;
        dx = -1;
    end

    y = y1;
    if y2 > y1
        yd = y2-y1;
        dy = 1;
    else
        yd = y1-y2;
        dy = -1;
    end

    p = [];

    if xd > yd
      a = 2*yd;
      b = a - xd;
      c = b - xd;

      while 1
          % test if x and y are inside map
          if (x < 0 || y < 0 || x > R || y > C)
              p = [inf inf];
              break
          end
        if map(x, y) == 1
            p= [x y];
            break
        end
        if all([x-x2 y-y2] == 0)
            p = [inf inf];
            break
        end
        if  b < 0
            b = b+a;
            x = x+dx;
        else
            b = b+c;
            x = x+dx; y = y+dy;
        end
      end
    else
      a = 2*xd;
      b = a - yd;
      c = b - yd;

      while 1
          % test if x and y are inside map
          if (x < 0 || y < 0 || x > C || y > R)
              p = [inf inf];
              break
          end
        if map(x, y) == 1
            p= [x y];
            break
        end
        if all([x-x2 y-y2] == 0)  % end of map without obstacle
            p = [inf inf];
            break
        end
        if  b < 0
            b = b+a;
            y = y+dy;
        else
            b = b+c;
            x = x+dx; y = y+dy;
        end
      end
    end
end
