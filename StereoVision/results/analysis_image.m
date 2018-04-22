
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analysis Image
% Task: Script to analyse different disparity images
% Author: Max Ronecker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
disp_1 = imread('full_disp_0.jpg');

surf(disp_1)
hold on 
image(disp_1,'CDataMapping','scaled')
 