function [trees] = findTrees(W,S,p,R,C)

% DEFINITION:
% Image processing algorithm to scan an occupancy grid, find circular
% shapes, record x,y coordinates and diameter.

% INPUTS:
% W: row width [m]; S: spacing between trees [m]; p: (x,y) coordinates of
% the first tree in the first row; R & C: Image size (rows and columns)
% [pixels].

% OUTPUTS:
% trees: 4xN vector containing the coordinates (x,y), diameter and label of
% each potential tree found in the occupancy grid.

%% SECTION 1 - Enhancing Image
global bitmap;

B = im2bw(bitmap,0.55);  % Threshold adopted after several trials
BW = edge(B,'Canny'); % Sharpens the image to enhance potential boundaries
se = strel('disk',4,0);
bitpos = imclose(BW,se); % Fill holes in the image given edges
[centers, radii, ~] = imfindcircles(bitpos,[1 30]); % Find circular shapes
% Rounds trunk diameters not to be greater than the maximum or smaller than the minimum
res = 42/R; % Image resolution [m/pixel]
radii = radii*res; % Converts radii from [pixels] to [m]

for i=1:length(radii)
    if radii(i) > 0.5
        radii(i) = 0.5;
    elseif radii(i) < 0.2
        radii(i) = 0.2;
    end
end
    
%% SECTION 2 - Find theoretical coordinates of trees centers
yC = zeros(1,5);
xC = zeros(1,7);

for i = 1:5
    [~,yC(i)] = XYtoIJ(W*(i-1)+p(1),-S*(i-1)+p(2)-2,42,42,R,C);
end

for i = 1:7
    [xC(i),~] = XYtoIJ(W*(i-1)+p(1),-S*(i-1)+p(2)+2,42,42,R,C);
end

minX = min(xC);
maxX = max(xC);
%% SECTION 2 - Group trees that belong a same row (1-5)
r1 = []; r2 = []; r3 = []; r4 = []; r5 = []; tolr = 6; tolB = 20;

for i = 1:length(radii)
    if centers(i,1) < yC(1) + tolr && centers(i,1) > yC(1) - tolr
        if centers(i,2) < maxX+tolB && centers(i,2) > minX-tolB
            r1 = [r1 i];
        end
    end
    if centers(i,1) < yC(2) + tolr && centers(i,1) > yC(2) - tolr
        if centers(i,2) < maxX+tolB && centers(i,2) > minX-tolB
            r2 = [r2 i];
        end
    end
    if centers(i,1) < yC(3) + tolr && centers(i,1) > yC(3) - tolr
        if centers(i,2) < maxX+tolB && centers(i,2) > minX-tolB
            r3 = [r3 i];
        end
    end
    if centers(i,1) < yC(4) + tolr && centers(i,1) > yC(4) - tolr
        if centers(i,2) < maxX+tolB && centers(i,2) > minX-tolB
            r4 = [r4 i];
        end
    end
    if centers(i,1) < yC(5) + tolr && centers(i,1) > yC(5) - tolr
        if centers(i,2) < maxX+tolB && centers(i,2) > minX-tolB
            r5 = [r5 i];
        end
    end    
end
%% SECTION 3 - Get coordinates and diameter of found trees
k = 1;

for i=1:length(r1)
    [x(k),y(k)] = IJtoXY(centers(r1(i),2),centers(r1(i),1),42,42,R,C);
    c(k) = radii(r1(i));
    nTree(k) = i;
    label(k) = 1;
    k = k+1;
end

for i=1:length(r2)
    [x(k),y(k)] = IJtoXY(centers(r2(i),2),centers(r2(i),1),42,42,R,C);
    c(k) = radii(r2(i));
    nTree(k) = i;
    label(k) = 2;
    k = k+1;
end

for i=1:length(r3)
    [x(k),y(k)] = IJtoXY(centers(r3(i),2),centers(r3(i),1),42,42,R,C);
    c(k) = radii(r3(i));
    nTree(k) = i;
    label(k) = 3;
    k = k+1;
end

for i=1:length(r4)
    [x(k),y(k)] = IJtoXY(centers(r4(i),2),centers(r4(i),1),42,42,R,C);
    c(k) = radii(r4(i));
    nTree(k) = i;
    label(k) = 4;
    k = k+1;
end

for i=1:length(r5)
    [x(k),y(k)] = IJtoXY(centers(r5(i),2),centers(r5(i),1),42,42,R,C);
    c(k) = radii(r5(i));
    nTree(k) = i;
    label(k) = 5;
    k = k+1;
end

%% SECTION 4 - Group all data and generate a txt file report
trees(:,1) = x;
trees(:,2) = y+10;
trees(:,3) = c;
trees(:,4) = nTree;
trees(:,5) = label;

trees(k,:) = trees(k-1,:);
trees(2:end,:) = trees(1:end-1,:);

fileID = fopen('Nursery Report.txt','w');
fprintf(fileID,'%.2f %.2f %.2f %d %d\n',trees);
dlmwrite('Nursery Report.txt',trees,'delimiter',' ');
fclose(fileID);

replaceLine = 1;
numLines = k;
newText = 'x_[m] y_[m] diameter_[m] M row_membership';

fileID = fopen('Nursery Report.txt','r');
mydata = cell(1, numLines);
for i = 1:numLines
   mydata{i} = fgetl(fileID);
end
fclose(fileID);

mydata{replaceLine} = newText;

fileID = fopen('Nursery Report.txt','w');
fprintf(fileID,'%s\n',mydata{:});
fclose(fileID);

t1 = trees(2:end,1);
t2 = trees(2:end,2);
t3 = trees(2:end,3);
trees = [t1 t2 t3];

clc;
