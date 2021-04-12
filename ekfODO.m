function [qNext, Pnext] = ekfODO(qPrev, Pprev, odo, V, W, qGPS, dt, k)
% Function Calculates best estimate of pose based upon odometry model and
% GPS data
%Inputs: qPrev is previous estimated state
%        Pprev is previous process noise covariance matrix
%        odo(1) is noisy change in distance between qNext and qPrev
%        odo(2) is noisy change in heading between qNext and qPrev
%        V is odometry noise covariance matrix
%        W is gps noise covariance matrix
%        qGPS is noisy gps x y theta data
%        dt is GPS sampling interval
%        k is loop interation step
%Outputs: qNext is next estimated state [3x1]
%         Pnext is next estimated process noise covariance matrix [3x3]
global DT %Controller sampling interval
 
    %Integrate Odometry Data
    qNext(1) = qPrev(1)+odo(1)*cos(qPrev(3)); %x
    qNext(2,1) = qPrev(2)+odo(1)*sin(qPrev(3)); %y
    qNext(3,1) = qPrev(3)+odo(2); %theta
    
    %Update Covariance Matrix
    Fx = [1 0 -odo(1)*sin(qPrev(3));0 1 odo(1)*cos(qPrev(3));0 0 1]; %State Jacobian --> Odometer
    Fv = [cos(qPrev(3)) 0;sin(qPrev(3)) 0;0 1]; %Noise Jacobian --> Odometer noise
    P = Pprev;
    P = Fx*P*Fx'+Fv*V*Fv';
    
    %Use GPS Data When Available
    if rem(k,dt/DT) == 0

        S = P+W;
        
        %Kalman Gain
        K = P/S;
        
        %Innovation
        v = qGPS - qNext; %Difference in state estimate from GPS and Odomoter
        
        %Update State Estimate
        qNext =  qNext + K*v;
        
        %Update Uncertainty Covariance
        P = P - K*P;
    end
    
    Pnext = P;
end