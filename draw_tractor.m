function tractor = draw_tractor(width,length)
%% Comments
% Creates a tractor shape based on the parameters width and legnth.
% Returns to the user the coordinates of all vertices of tractor and the 
% center as well.

%Creates a tractor object with a the left rear wheel located at (0,0)
[tractor] = [0,0;0,0.75*length;0.3*width,0.75*length;0.3*width,length;0.7*width,length;0.7*width,0.75*length;width,0.75*length;width,0;0,0;0,0];
%Translates the object so that the center of the tractor is located at
%(0,0)
tractor (:,1) = tractor(:,1) - width/2;
tractor (:,2) = tractor(:,2)-length/2;

%Add wheels to the tractor
%[wheels] = [0,0;0.3*width,length;0.7*width,length;width,0];

%Roates the object by 270
theta = deg2rad(270);
T = [cos(theta),- sin(theta);sin(theta),cos(theta)];
for i=1:9
    tractor(i,:) = T*tractor(i,:)'; 
end
%Add the center location to the object coordinates vector
tractor(10,1) = 0;
tractor(10,2) = 0;

%Rotates wheels as well
% for i=1:4
%     wheels(i,:) = T*wheels(i,:)';
% end

%% Ploting/Debbuging
% x=tractor(:,1);
% y=tractor(:,2);
% plot(x,y,'r-');
% hold on;
% xw=wheels(:,1);
% yw=wheels(:,2);
% plot(xw,yw,'ko');

% 
% [wheels] = [0,r/2;-e,r/2;-e,-r/2;0,-r/2;0,0];
% xw = wheels(:,1);
% yw = wheels(:,2);
% plot(xw,yw,'r-');
% 
% [wheels2] = [width,radius/2;width+e,radius/2;width+e,-radius/2;width,-radius/2;width,0];
% xw2 = wheels2(:,1);
% yw2 = wheels2(:,2);
% % plot(xw2,yw2,'r-');
% 
% [wheels3] = [0.3*width,length-radius/2;0.3*width-e,length-radius/2;0.3*width-e,length+radius/2;0.3*width,length+radius/2;0.3*width,length];
% xw3 = wheels3(:,1);
% yw3 = wheels3(:,2);
% % plot(xw3,yw3,'r-');
% 
% [wheels4] = [0.7*width,length-radius/2;0.7*width+e,length-radius/2;0.7*width+e,length+radius/2;0.7*width,length+radius/2;0.7*width,length];
% xw4 = wheels4(:,1);
% yw4 = wheels4(:,2);
% % plot(xw4,yw4,'r-');