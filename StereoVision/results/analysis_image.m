
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analysis Image
% Task: Script to analyse different disparity images
% Author: Max Ronecker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
disp_full = imread('full_disp_0.jpg');
disp_no_gnd = imread('ground_rm_disp_0.jpg');
disp_no_sky = imread('sky_rm_disp_0.jpg');
disp_ready = imread('stixel_ready_16.jpg');

%% Plot Full Disparity
fig1 = figure('Name','Full Disparity');
surf(disp_full,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_full,'CDataMapping','scaled');

hold off

%% Plot Ground_removed Disparity
fig2 = figure('Name','Ground removed');
surf(disp_no_gnd,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_no_gnd,'CDataMapping','scaled');

%% Plot Sky removed Disparity
fig3 = figure('Name','Sky removed');
surf(disp_no_sky,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_no_sky,'CDataMapping','scaled');

%% Plot Stixel ready Disparity
fig4 = figure('Name','Stixel ready');
surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_ready,'CDataMapping','scaled');

%% Delete non-plausible Values
mean_image_all = mean(disp_ready(:));
std_all = std(im2single(disp_ready(:)));
hist_all = figure('Name','Histogram all');
histogram(disp_ready(:),15);

zero_mask = disp_ready >0;
mean_no_zero = mean(disp_ready(zero_mask>0));
std_no_zero = std(im2single(disp_ready(zero_mask>0)));
hist_no_zero = figure('Name','Histogram no zero');
histogram(disp_ready(zero_mask>0));


%x_all = 0:0.1:25;
%factor = normpdf(mean_image_all,mean_image_all,std_all);
%normal_all = normpdf(x_all,mean_image_all,std_all);
%norm_all = figure('Name','Normal distribution all');
%plot(x_all,normal_all);

%x_zero = 75:0.1:125;
%normal_no_zero = normpdf(x_zero,mean_no_zero,std_no_zero);
%norm_zero = figure('Name','Normal distribution no zero');
%plot(x_zero,normal_no_zero);


disp_ready_ada = disp_ready;
disp_ready_ada(disp_ready > mean_no_zero) = 0;


% Plot Stixel ready Disparity
fig5 = figure('Name','Stixel adapted');
surf(disp_ready_ada,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
hold on 
image(disp_ready_ada,'CDataMapping','scaled')

 