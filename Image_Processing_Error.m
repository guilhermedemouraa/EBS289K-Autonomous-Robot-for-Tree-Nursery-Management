%EBS 289K Final Project - Image Processing Analysis
%Students: Bennett Evans, Guilherme De Moura Araujo & Nicolas Buxbaum
%Professor: Stavros G. Vougioukas

%trees = findTrees(3,2,[18.5 20],R,C);
global nursery;
%%
k = 1;
% t1 = trees(2:end,1);
% t2 = trees(2:end,2);
% t3 = trees(2:end,3);
% t = [t1 t2 t3];
for i = 1:length(trees)
    EUD = 100;
    for j = 1:length(nursery)
        dx = (trees(i,1)-nursery(j,1))^2;
        dy = (trees(i,2)-nursery(j,2))^2;
        dist = sqrt(dx+dy);
        if dist < EUD
            error(k) = dist;
            EUD = dist;
            cerror(k) = abs(trees(i,3) - nursery(j,3));
        end
    end
    k = k+1;
end

%% Tree location stats
histogram(error,5)
xlabel('Error (m)')
title('Tree center location error'); 
figure()
plot(1:length(error),error,'b');
xlabel('Number of tree');
ylabel('Error (m)');
mean_error = mean(error);
x1 = 1; x2 = length(error);
x_mean = [x1 x2];
y_mean = [mean_error mean_error];
hold on;
plot(x_mean,y_mean,'r-');
rmse_error = rms(error);
y_rmse = [rmse_error rmse_error];
plot(x_mean,y_rmse,'g-');
legend('Error (m)','Mean error (m)', 'RMSE error (m)');

max_error = max(error);
percentile_error = prctile(error,95);

fprintf('The maximum error in tree location is %.3f m\n', max_error);
fprintf('The 95th percentile error in tree location is %.3f m\n', percentile_error);
fprintf('The RMSE of error in tree location is %.3f m\n', rmse_error);

%% Tree diameter stats
figure()
histogram(cerror,5)
xlabel('Error (m)')
title('Tree diameter error'); 
figure();
plot(1:length(cerror),cerror,'b');
xlabel('Number of tree');
ylabel('Diameter error (m)');
mean_error = mean(cerror);
x1 = 1; x2 = length(cerror);
x_mean = [x1 x2];
y_mean = [mean_error mean_error];
hold on;
plot(x_mean,y_mean,'r-');
rmse_error = rms(cerror);
y_rmse = [rmse_error rmse_error];
plot(x_mean,y_rmse,'g-');
legend('Diameter Error (m)','Mean error (m)', 'RMSE error (m)');

max_error = max(cerror);
percentile_error = prctile(cerror,95);

fprintf('The maximum error in diameter estimation is %.3f m\n', max_error);
fprintf('The 95th percentile error in diameter estimation is %.3f m\n', percentile_error);
fprintf('The RMSE of error in diameter estimation is %.3f m\n', rmse_error);

%% DEBUGGING
% plot(nursery(:,1),nursery(:,2),'o')
% hold on
% plot(trees(:,1),trees(:,2),'o')
% legend('Nursery','Trees');