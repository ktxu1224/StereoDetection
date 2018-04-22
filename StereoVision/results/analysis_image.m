
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analysis Image
% Task: Script to analyse different disparity images
% Author: Max Ronecker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
disp_full = imread('full_disp_0.jpg');
disp_no_gnd = imread('ground_rm_disp_0.jpg');
disp_no_sky = imread('sky_rm_disp_0.jpg');
disp_ready = imread('after_morph_disp_0.jpg');

% Plot Full Disparity
fig1 = figure('Name','Full Disparity');
surf(disp_full,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_full,'CDataMapping','scaled')

hold off

% Plot Ground_removed Disparity
fig2 = figure('Name','Ground removed');
surf(disp_no_gnd,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_no_gnd,'CDataMapping','scaled')

% Plot Sky removed Disparity
fig3 = figure('Name','Sky removed');
surf(disp_no_sky,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
hold on 
image(disp_no_sky,'CDataMapping','scaled')

% Plot Stixel ready Disparity
fig4 = figure('Name','Stixel ready');
surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
hold on 
image(disp_ready,'CDataMapping','scaled')


disp_ready_ada = disp_ready;
disp_ready_ada(disp_ready > 80) = 0;

% Plot Stixel ready Disparity
fig4 = figure('Name','Stixel adapted');
surf(disp_ready_ada,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
hold on 
image(disp_ready_ada,'CDataMapping','scaled')

 