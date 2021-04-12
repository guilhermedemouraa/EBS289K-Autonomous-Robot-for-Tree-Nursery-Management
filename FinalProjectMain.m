%EBS 289K Final Project
%Students: Bennett Evans, Guilherme De Moura Araujo & Nicolas Buxbaum
%Professor: Stavros G. Vougioukas

%% Initialize Robot and Simulation Parameters
clear all; clc;
generateNursery; %Creates a randon nursery

global DT
global dT
global bitmap

%Simulation Parameters
T = 120; %simulation run time (s), Very big to produce large sample sizes
DT = 0.1; %controller sampling rate
dT = 0.01;
dt = 1; %GPS sampling rate
N = T/DT;
huge = 10000000;

%Vehicle Parameters
u(1) = 0; %Initial steering command
u(2) = 0; %Initial velocity command
L = 3; %Wheelbase
w = 2; %Vehicle width
tau_v = 0.2; %Velocity lag
tau_gamma = 0.1; %Steering lag
umax(2,1) = 2; %Max linear velocity
umax(1) = deg2rad(55); %Max steering angle
umin(2,1) = -umax(2);
umin(1) = -1*umax(1);
Qmax(1) = huge;
Qmax(2) = huge;
Qmax(3) = huge;
Qmax(4) = huge;
Qmax(5) = huge;
Qmin(1) = -huge;
Qmin(2) = -huge;
Qmin(3) = -huge;
Qmin(4) = -huge;
Qmin(5) = -huge;
Ld = 1.5; %look ahead distance (m)
rMin = L/atan(umax(1)); %Minimum turning radius

bitmapTrue = bitmap;
addpath(genpath('geom2d'));
global bitodds;
global rangeMax;

%lidar values
rangeMax = 20; angleSpan = pi; angleStep = angleSpan/720; 

Xmax = 50; Ymax = 50; %physical dimensions of space (m)

R = 350; C = 350; %rows and columns; discretization of physical space in a grid
bitmap = 0.0* ones(R, C); %initialize as empty
%% Generate Field and Path
%Define Field Coordinates
K = 10; %Number of Rows
RW = 3; %Row Width
RL = 20; %Row Length
startEnd = [0;0]; %Coordinates of start/end 
Field = zeros(2,3*K+2); %Each row gets 3 points, one at each end and one in the center
Field(:,1) = startEnd;
Field(:,end)= startEnd;
Field(:,2) = [20; 20]; %Coordinates of First Field Point

for k = 3:1:K*3+1
    if rem(k,3) == 2
        Field(1,k) = Field(1,k-1)+RW;
        Field(2,k) = Field(2,2);
    elseif rem(k,3) == 0 || rem(k,3) == 1
        Field(1,k) = Field(1,k-1);
        Field(2,k) = Field(2,k-1) + RL/2;
    end
end

%Row Indices
%       |4|7|   |3n+1
%       | | |   |
%       | | |   |
% 1&3n+2|3|6|   |3n
%       | | |   |
%       | | |   |
%       |2|5|...|3n-1


%% Create Cost Matrix
huge = 10000000000;
dMat = zeros(length(Field));
for i = 1:length(dMat)
    for j = 1:length(dMat)
        d = abs(round(i/3)-round(j/3)); %Calculates difference of row number for 2 points. Divide by two because there are 2 points on each row
    if i == j || (i == 1 && j == 3*K+2) || (j == 1 && i == 3*K+2) || ((rem(i,3) == 0 || rem(j,3) == 0) && (i == 1 || i == 3*K+2 || j == 1 || j == 3*K+2))||(rem(i,3) == 0 && rem(j,3) == 0)
        dMat(i,j) = huge;
    elseif i == 1 || i == 3*K+2 || j == 1 || j == 3*K+2 %Is i or j the start/end point & it's counterpart not within a row
        dMat(i,j) = abs(Field(1,i)-Field(1,j)) + abs(Field(2,i)-Field(2,j)); %Manhattan Distance
    elseif d == 0 % Are i and j on the same row
        dMat(i,j) = 0;
    elseif Field(2,i) ~= Field(2,j) %Are i and j on different rows and different sides of the field
        dMat(i,j) = huge;
    else %Otherwise i and j are on different rows but the same side of the field
     
        if d*RW >= 2*rMin %If seperation is greater than minimum turning diameter
            dMat(i,j) = d*RW + (pi-2)*rMin; %Than a pi turn is possible
        else %Otherwise an omega turn must be made
            dMat(i,j) = 3*pi*rMin-2*rMin*acos(1-(2*rMin+d*RW)^2/(8*rMin^2));
        end
    end
    end
end

%% Run TSP Algorithm

Field = Field';
t = cputime;
resultStruct= tspof_ga('xy', Field , 'DMAT', dMat, 'SHOWRESULT', false, 'SHOWWAITBAR', false, 'SHOWPROG', false);
E = cputime - t; %print time required to compute it.
route = [1 resultStruct.optRoute]; % extract node sequence
minDist = resultStruct.minDist; %print computed minimum distance21

%% Generate Path
dx = 0.1; %Path point seperation
PathX = zeros(1,5000);
PathY = PathX;
i = 1; %Current Index
if Field(route(2),2) ~= 20 %Enter at top of field
    %Vertical
    j = i+abs(Field(route(2),2)-(Field(1,2)))/dx; %End Index
    PathY(i:j) = linspace(Field(1,2),Field(route(2),2),j-i+1); %Vehicle first travels Y direction to be level with first point
    PathX(i:j) = Field(1,1)*ones(1,j); %Corresponding X points
    i = j;
    %Turn into Field 
    [m,PathX(i:i+m-1),PathY(i:i+m-1)] = piTurn([PathX(j),PathY(j)],Field(route(2),:),dx,rMin);
    i = i + m - 1;
else %Enter at bottom of field

    %Vertical
    j = i+abs(Field(route(2),2)-(Field(1,2)-2*ceil(rMin)))/dx; %End Index
    PathY(i:j) = linspace(Field(1,2),(Field(route(2),2)-2*ceil(rMin)),j-i+1); %Vehicle first travels Y direction to be level with first point
    PathX(i:j) = Field(1,1)*ones(1,j); %Corresponding X points
    i = j;

    %Right Turn
    a = pi:-dx:pi/2;
    turnX = Field(1,1)+ceil(rMin) + ceil(rMin)*cos(a);
    turnY = PathY(j) + ceil(rMin)*sin(a);
    j = j + length(a)-1;

    PathX(i:j) = turnX;
    PathY(i:j) = turnY;
    i = j;

    %Horizantal
    j = j + (Field(route(2),1)-(Field(1,1)+2*ceil(rMin)))/dx;

    PathX(i:j) = linspace((Field(1,1)+ceil(rMin)),(Field(route(2),1)-ceil(rMin)),j-i+1); %Travels horizantilly to first field point
    PathY(i:j) = (Field(route(2),2)-ceil(rMin))*ones(1,j-i+1); %Corresponding X points
    i = j;
    
    %Left Turn
    a = 3*pi/2:dx:2*pi;
    turnX = Field(route(2),1)-ceil(rMin) + ceil(rMin)*cos(a);
    turnY = Field(route(2),2) + ceil(rMin)*sin(a);
    j = j + length(a)-1;

    PathX(i:j) = turnX;
    PathY(i:j) = turnY;
    i = j;
    
end

%Path In Field
for p = 2:length(route)-1 %Start Path building loop at first point on field
    if Field(route(p),1) == Field(route(p+1),1)%True if points are on same row
        dy = abs(Field(route(p+1),2)-Field(route(p),2));
        j = i+dy/dx;
        PathY(i:j) = linspace(Field(route(p),2),Field(route(p+1),2),j-i+1);
        PathX(i:j) = PathX(i)*ones(1,j-i+1);
        i = j;
    else
        d = abs(round(route(p)/3)-round(route(p+1)/3));
        if d*RW >= 2*rMin %If seperation is greater than minimum turning diameter
            %Pi turn
            %[m,~,~] = piTurn(Field(route(p),:),Field(route(p+1),:),dx,rMin);
            [m,PathX(i:i+m-1),PathY(i:i+m-1)] = piTurn(Field(route(p),:),Field(route(p+1),:),dx,rMin);
            i = i + m - 1;
        else
            %[m,~,~] = omegaTurn(Field(route(p),:),Field(route(p+1),:),rMin,d*RW);
            [m,PathX(i:i+m-1),PathY(i:i+m-1)] = omegaTurn(Field(route(p),:),Field(route(p+1),:),rMin,d*RW);
            i = i + m - 1;
        end
    end
end

%Path to End
%Construct Turn From Last Route Point to End So Vehicle Doesn't Lose Path

if Field(route(end),2) ~= 20 %Exit at top of field
    [m,PathX(i:i+m-1),PathY(i:i+m-1)] = piTurn(Field(route(end),:),[Field(end,1),Field(route(end),2)],dx,rMin);
    i = i + m - 1;
    %Vertical
    j = i+abs(Field(route(end),2)-Field(end,2))/dx; %End Index
    PathY(i:j) = linspace(Field(route(end),2),Field(end,2),j-i+1); %Vehicle first travels Y direction to be level with first point
    PathX(i:j) = Field(end,1)*ones(1,j-i+1); %Corresponding X points
    i = j;
else %Exit at bottom of field

    %Right Turn
    a = 0:-dx:-pi/2;
    turnX = Field(route(end),1)-ceil(rMin) + ceil(rMin)*cos(a);
    turnY = Field(route(end),2) + ceil(rMin)*sin(a);
    j = j + length(a)-1;

    PathX(i:j) = turnX;
    PathY(i:j) = turnY;
    i = j;

    %Horizantal
    j = j + (Field(route(end),1)-(Field(end,1)+2*ceil(rMin)))/dx;

    PathX(i:j) = linspace((Field(route(end),1)-ceil(rMin)),(Field(end,1)+ceil(rMin)),j-i+1); %Travels horizantilly to first field point
    PathY(i:j) = (Field(route(end),2)-ceil(rMin))*ones(1,j-i+1); %Corresponding X points
    i = j;
    
    %Left Turn
    a = pi/2:dx:pi;
    turnX = Field(end,1)+ceil(rMin) + ceil(rMin)*cos(a);
    turnY = Field(route(end),2)-2*ceil(rMin) + ceil(rMin)*sin(a);
    j = j + length(a)-1;

    PathX(i:j) = turnX;
    PathY(i:j) = turnY;
    i = j;
    
    %Vertical
    j = i+abs(Field(route(end),2)-(Field(end,2)-2*ceil(rMin)))/dx; %End Index
    PathY(i:j) = linspace((Field(route(end),2)-2*ceil(rMin)),Field(end,2),j-i+1); %Vehicle first travels Y direction to be level with first point
    PathX(i:j) = Field(end,1)*ones(1,j-i+1); %Corresponding X points
    i = j;
end

j = j+20; %Here I add overflow points so robot doesn't lose path

PathY(i:j) = linspace(Field(end,2),(Field(end,2)-dx*20),j-i+1); %Vehicle first travels Y direction to be level with first point
PathX(i:j) = PathX(i)*ones(1,j-i+1); %Corresponding X points
Path = [PathX;PathY];

%% Have Robot Follow Path and Scan Field

%Initialize Covariance Arrays
V = [0.0025 0;0 0.00011062]; %odometry covariance
W = [0.0011 0 0;0 0.00092558 0;0 0 0.00041117]; %GPS covariance
P = zeros(3); %Assume initial pose is perfectly known ie zero covariance matrix

%Initialize State Arrays
X_true = zeros(N,5);
%odo = zeros(N-1,2);
X_kalman = zeros(N,3)';
X_gps = zeros(1,3)';
pathError = zeros(1,N-1);

%Initial Pose
X_true(1,3) = pi/2;
X_true(5,1) = umax(2);
X_gps(3) = X_true(1,3);
X_kalman(3,1) = X_true(1,3);

%Speed Control
u(2) = umax(2);

%Run Simulation w/ EKF
m = 5;
j = p+10;
for k = 1:N-1
    %Give Pure Pursuit Control Input
    [u(1),~,I] = purePursuit(X_kalman(:,k),L,Ld,Path(:,m:j));
    
    %Feed Driving Input to Controller
    [X_true(k+1,:), odo] = robot_odo(X_true(k,:),u,umin,umax,Qmin,Qmax,L,tau_gamma,tau_v);
    
    %Take GPS data available every 1 second
    if rem(k,dt/DT) == 0 %|| k == 1
        [X_gps(1,1),X_gps(2,1),X_gps(3,1)] = GPS_CompassNoisy(X_true(k+1,1),X_true(k+1,2),X_true(k+1,3));
    end
    
    %Run EKF
    [X_kalman(:,k+1), P] = ekfODO(X_kalman(:,k), P, odo, V, W, X_gps, dt, k);
    
    if rem(k,dt/DT) == 0
        %Only scan when x & y are positive because bitmap has default zero
        %minimums
        if X_kalman(1,k+1) > 0 && X_kalman(2,k+1) > 0 
            %Scanner    
            Tl = SE2([X_kalman(1,k+1) X_kalman(2,k+1) X_kalman(3,k+1)]);
            p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl.T, bitmapTrue, Xmax, Ymax);
            p2(1) = p(1,2); % Implementation of signal filtering
            p2(2) = p(2,2); % Implementation of signal filtering
            for i=3:length(p)-2
                A = [p(i-2,2),p(i-1,2),p(i,2),p(i+1,2),p(i+2,2)];
                p2(i) = median(A); %Low pass filter
            end
            p2(i+1) = p(end-1,2);
            p2(i+2) = p(end,2);
            p2 = p2';
            %p(:,2) = p2;
            for i=1:length(p)
                angle = p(i,1); range = p2(i);
                % handle infinite range
                if(isinf(range)) 
                    range = rangeMax+1;
                end
                n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax, 0.75);
            end
        end
    end
    m = m+I-1; %Give Pure Pursuit controller path from previous goal point +10 points
    j = m+10;

    if k > 100 %When sufficiently far from the start point to not inadvertantly select it as the goal point
   
        d = sqrt((X_kalman(1,k)-Path(1,1))^2+(X_kalman(2,k)-Path(2,1))^2); %Take advantage of the fact that the start and end point are the same
        if d < 0.5
            break
        end
    end
  %Find minimum seperation from Path to Calculate Error
   seperation = sqrt((Path(1,:)-X_kalman(1,k+1)).^2+(Path(2,:)-X_kalman(2,k+1)).^2); %Minimum Distance Between Vehicle Position and Path Points
   pathError(k) = min(seperation); %i = index of smallest value of seperation
end
rmse_Path = rms(pathError);
mean_Path = mean(pathError);
max_Path = max(pathError);
figure;
plot(PathX,PathY,X_kalman(1,:),X_kalman(2,:))
title('Robot Path and Planned Path')
legend('Intended Path','Robot Path','location','southeast')
figure;
histogram(pathError(1:k))
title('Distribution of Minimum Robot Seperation from Path')
xlabel('Robot Seperation From Path (m)')
%% Image Post Processing
S = 2;
p = [18.5 20];

for i=1:R
    for j=1:C
        bitmap(i,j) = bitodds(i,j)/(1+bitodds(i,j));
    end
end
trees = findTrees(RW,S,p,R,C);

%% Plot Robot Path With Tree Centers

figure;
hold on;
plot(PathX,PathY,X_kalman(1,:),X_kalman(2,:))
title('Robot Path and Trees')
plot(-0,55,'color',[0.4660 0.6740 0.1880],'marker','o');
xlim([-5 35])
ylim([-10 50])
centers = [trees(:,1) trees(:,2)]; radii = trees(:,3);
viscircles(centers, radii,'Color',[0.4660 0.6740 0.1880]);
legend('Intended Path','Robot Path','Trees','location','southeast')
axis equal
axis square
hold off