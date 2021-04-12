function p = updateLaserBeamGrid(angle, range, Tl, R, C, Xmax, Ymax, a)

% DEFINITION:
% Function assumes a laser scanner with a pose in world coordinates defined
% by Tl. It shoots rays from -angleSpan/2 to +angleSpan/2 with step
% angleStep. Given a bitmap (boolean occupancy grid with obstacles) with
% origin at (0,0) and uper NE corner (Xmax, Ymax), the result is an
% occupancy map with probabilities of pixel being full.

% INPUTS: angle = angle span from laser scanner [deg]; range = maximum
% distance reached by the laser scanner [m]; Tl = Transfrom from laser
% scanner to world frame; R and C = Rows and Columns -> discretization of
% physical space in a grid [pixels]; Xmax and Ymax: Physical dimensions of
% space [m]; a = sensor accuracy.

% OUTPUTS:
% p = Number of updated pixels; bitmap = occupancy grid of field

%% 
global bitodds; % Local variable that stores the odds of each pixel in the map
oddsfull = a/(1-a);
oddsempty = (1-a)/a;

if isempty(bitodds)
    bitodds = 1*ones(R,C); % Starts with p = 50% -> Odds = 1;
end

% Transform laser origin to world frame
P1 = Tl*[0 0 1]';
x1=P1(1);     y1=P1(2);

if (isinf(range)) % Handle Inf return values
    range = Xmax^2+Ymax^2;  % Assign arbitrary huge value
end

% First produce target point for laser in scanner frame
Xl = range * cos(angle);
Yl = range * sin(angle);

% Transform target point in world frame
P2 = Tl*[Xl Yl 1]';
x2=P2(1); y2=P2(2);

% Clip laser beam to boundary polygon so that 'infinite' (rangeMax) range
% extends up to the boundary
dy = y2-y1; dx = x2-x1;
% ATTENTION: if dy or dx is close to 0 but negative, make it almost zero
% positive
if (abs(y2-y1)) < 1E-6
    dy = 1E-6;
end
edge = clipLine([x1,y1,dx,dy],[0 Xmax 0 Ymax]);
% Laser origin is always inside map
% Decide if clipping is necessary
l1 = sqrt( (x1-edge(3))^2 + (y1 - edge(4))^2);
if range >= l1
    x2 = edge(3); y2 = edge(4);
end

% Map world points to integer coordninates
[ I1, J1 ] = XYtoIJ(x1, y1, Xmax, Ymax, R, C); % laser source
[ I2, J2 ] = XYtoIJ(x2, y2, Xmax, Ymax, R, C); % obstacle pixel

% Update detected obstacle pixel
bitodds(I2, J2) = bitodds(I2, J2)*oddsfull;
% use bresenham to find all pixels that are between laser and obstacle
l=bresenhamFast(I1,J1,I2,J2);
%[l1 l2]=size(l);
for k=1:length(l)-1 %skip the target pixel
     bitodds(l(k,1),l(k,2)) = bitodds(l(k,1),l(k,2))*oddsempty; % free pixels
end

p = length(l) + 1;  % Number of updated pixels

% Converts odds in probability
% for i=1:R
%     for j=1:C
%         bitmap(i,j) = bitodds(i,j)/(1+bitodds(i,j));
%     end
% end

