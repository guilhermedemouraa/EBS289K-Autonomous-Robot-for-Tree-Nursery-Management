%% Covariance Estimate
%Run Odometry model to estimate covariance of odometry and gps noise
clear all

global DT
global dT

%Simulation Parameters
T = 1000; %simulation run time (s), Very big to produce large sample sizes
DT = 0.01; %controller sampling rate
dT = 0.01;
N = T/DT;
huge = 10000000;

%Vehicle Parameters
u(1) = 0; %Initial steering command
u(2) = 0; %Initial velocity command
L = 3; %Wheelbase
w = 2; %Vehicle width
tau_v = 0.2; %Velocity lag
tau_gamma = 0.1; %Steering lag
umax(2,1) = 5; %Max linear velocity
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


%Initialize State Arrays
X_true = zeros(5,N);
%odo = zeros(2,N-1);
X_odo = zeros(3,N);
X_gps = zeros(4,N/100);
qPrev = X_true(:,1);
odo = [0 0];

%Run Simulation
m = 2;
for k = 1:N-1
    %Have Robot Run on Straight Path
    u=[(randn*(90*pi/180)) 5*randn];
  
    
    %Feed Driving Input to Controller
    [qNext, odo] = robot_odo(qPrev,u,umin,umax,Qmin,Qmax,L,tau_gamma,tau_v);
    
    %Calculate Odometry Error
    dThetaTrue = qNext(3)-qPrev(3);
    dDistTrue = sqrt((qNext(1)-qPrev(1))^2+(qNext(2)-qPrev(2))^2);
    distEodo(k) = dDistTrue-odo(1);
    thetaEodo(k) = dThetaTrue - odo(2);
    
    %Integrate Odometry Data
    X_odo(3,k+1) = X_odo(3,k)+odo(2); %New heading equals previous heading plus change in angle
    X_odo(1,k+1) = X_odo(1,k)+odo(1)*cos(X_odo(3,k));
    X_odo(2,k+1) = X_odo(2,k)+odo(1)*sin(X_odo(3,k));
    
    %Take GPS data available every ten sampling intervals
    if rem(k,100) == 0 %|| k == 1
        [X_gps(1,m),X_gps(2,m),X_gps(3,m)] = GPS_CompassNoisy(X_true(1,k),X_true(2,k),X_true(3,k));
        X_gps(4,m) = k; %For debugging purposes
        m = m+1;
    end
    qPrev = qNext;
    X_true(:,k+1) = qNext;
end

%Plot Paths
figure;
plot(X_true(1,:),X_true(2,:),X_gps(1,:),X_gps(2,:),X_odo(1,:),X_odo(2,:));
legend('Ground Truth','GPS','Odometry')

%% Calculate True Odometry Readings
%deltaTheta = zeros(N-1,1);
%deltaY = zeros(N-1,1);
%deltaX = zeros(N-1,1);
%for k = 1:N-1
%    deltaY(k) = (X_true(k+1,2)-X_true(k,2)).^2;
 %   deltaX(k) = (X_true(k+1,1)-X_true(k,1)).^2;
 %  deltaTheta(k,1) = X_true(k+1,3) - X_true(k,3);
%end
%deltaTheta = deltaTheta';
%odoTrueDist = sqrt(deltaX+deltaY);
%odoTrueDist = odoTrueDist';

%Calculate Odometry Error to estimate noise variance
figure;
%distEodo = odoTrueDist(:,1)-odo(:,1);
histogram(distEodo)
title('Odometry Distance Noise PDF')
xlabel('Error (m)')
distVar = var(distEodo);
figure;
%thetaEodo = odo(:,2)-deltaTheta;
histogram(thetaEodo);
title('Odometry Angle Noise PDF')
xlabel('Error (rad)')
dThetaVar = var(thetaEodo);

%% Calculate GPS Error
m = 2;
trueGPS = zeros(4,length(X_gps));
for k = 1:N-1
    if rem(k,100) == 0
        trueGPS(1:3,m) = X_true(1:3,k);
        trueGPS(4,m) = k; %For debugging purpose
        m = m + 1;
    end
end
xEgps = trueGPS(1,:) -  X_gps(1,:);
yEgps = trueGPS(2,:) -  X_gps(2,:);
thetaEgps = trueGPS(3,:) -  X_gps(3,:);
figure;
histogram(xEgps,20)
title('GPS x Noise PDF')
xlabel('Error (m)')
xVar = var(xEgps);
figure;
histogram(yEgps,20)
title('GPS y Noise PDF')
xlabel('Error (m)')
yVar = var(yEgps);
figure;
histogram(thetaEgps,20)
title('GPS Heading Noise PDF')
xlabel('Error (rad)')
thetaVar = var(thetaEgps);
        
