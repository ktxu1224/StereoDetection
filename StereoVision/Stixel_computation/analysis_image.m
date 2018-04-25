
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Analysis Image
% Task: Script to analyse different disparity images
% Author: Max Ronecker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
disp_full = imread('full_disp_0.jpg');
%disp_no_gnd = imread('ground_rm_disp_0.jpg');
%disp_no_sky = imread('sky_rm_disp_0.jpg');
disp_ready = imread('stixel_ready_0.jpg');
normal_image = imread('001036.bmp');
%% Create Stixels

test_image = zeros(900,1570);

stixel_width = 1;
stixel_matrix = zeros(3,1570/stixel_width); % base_col, start_row, end_row, depth;
width = 1;
col_test = zeros(1,1570);
for col = 1: size(test_image,2)
    non_zero_flag = 0;
    
    % Search for Start row
    row = 900;
    while(non_zero_flag == 0 &&  row >=1)
        if(disp_ready(row,col)>1)
            test = 1;
            col_test(1,col) = 1;
            non_zero_flag = 1;
            stixel_matrix(1,col) = row;

            
        end
        row = row - 1;  
    end
    
    
    
end

%% Add height
max_height = 30;
for i = 1:size(stixel_matrix,2)
    if(stixel_matrix(1,i)>max_height)
        stixel_matrix(2,i) = stixel_matrix(1,i)-30;
   
    end
end

for col = 1:size(test_image,2)
    height = 0;
    row = stixel_matrix(1,col)+1;
    depth = 0;
    while(height <= max_height && row >=1)
        
        
        if(disp_ready(row,col) > 0)
            depth = [depth disp_ready(row,col)];
        end
        row = row - 1;
        height = height +1;
    end
    stixel_matrix(3,col) = mean(depth(:));
    
end
    
    





%% Zero Padding

 
 fig4 = figure('Name','Stixel ready');
 surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
 hold on 
 image(disp_ready,'CDataMapping','scaled');
 plot(stixel_matrix(1,:),'Color','r','LineWidth',2)
 plot(stixel_matrix(2,:),'Color','b','LineWidth',2)

stixel_width = 10; 
 
start_v = stixel_matrix(1,:);
start = conv(start_v,ones(1,stixel_width),'valid');
start_reduced = start(1:stixel_width:end)/stixel_width;

height_v = stixel_matrix(2,:);
height = conv(start_v,ones(1,stixel_width),'valid');
height_reduced = height(1:stixel_width:end)/stixel_width;

depth_v = stixel_matrix(3,:);
depth = conv(depth_v,ones(1,stixel_width),'valid');
depth_reduced = depth(1:stixel_width:end)/stixel_width;

compressed_stixel_matrix = [start_reduced;height_reduced;depth_reduced];


stixel_matrix_smooth = imresize(compressed_stixel_matrix, [3 1571], 'nearest');
col_pad = [1:1:1571];
stixel_matrix_smooth = round([stixel_matrix_smooth; col_pad]);



 fig5 = figure('Name','Stixel depth');
 surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
 hold on 
 image(disp_ready,'CDataMapping','scaled');
 plot(stixel_matrix_smooth(3,:),'Color','b','LineWidth',2)
 hold off
 
 %% Create Stixel Image

stixel_image = zeros(900,1571);

for col = 1:size(stixel_image,2)
    if(stixel_matrix_smooth(1,col) > max_height)
        for h = 0:max_height
            row_index= stixel_matrix_smooth(1,col)+1-h;
            stixel_image(row_index,col) = stixel_matrix_smooth(3,col);
            
            
        end
    end
    
end

 fig6 = figure('Name','Stixel depth 3D');
 surf(stixel_image,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
 hold on 
 image(disp_ready,'CDataMapping','scaled');
 
 fig7 = figure('Name','Real 3D');
 surf(-stixel_image,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
 hold on 
 image(normal_image);
 
 
 