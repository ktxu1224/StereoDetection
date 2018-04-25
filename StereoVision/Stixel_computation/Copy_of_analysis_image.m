
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

%% Plot Full Disparity
% fig1 = figure('Name','Full Disparity');
% surf(disp_full,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
% hold on 
% image(disp_full,'CDataMapping','scaled');
% 
% hold off
% % 
% % %% Plot Ground_removed Disparity
% % fig2 = figure('Name','Ground removed');
% % surf(disp_no_gnd,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
% % hold on 
% % image(disp_no_gnd,'CDataMapping','scaled');
% % 
% % %% Plot Sky removed Disparity
% % fig3 = figure('Name','Sky removed');
% % surf(disp_no_sky,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
% % hold on 
% % image(disp_no_sky,'CDataMapping','scaled');
% % 
% %% Plot Stixel ready Disparity
%fig4 = figure('Name','Stixel ready');
%surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
%hold on 
%image(disp_ready,'CDataMapping','scaled');
% 
% %% Delete non-plausible Values
% mean_image_all = mean(disp_ready(:));
% std_all = std(im2single(disp_ready(:)));
% hist_all = figure('Name','Histogram all');
% histogram(disp_ready(:),15);
% 
% zero_mask = disp_ready >0;
% mean_no_zero = mean(disp_ready(zero_mask>0));
% std_no_zero = std(im2single(disp_ready(zero_mask>0)));
% hist_no_zero = figure('Name','Histogram no zero');
% histogram(disp_ready(zero_mask>0));
% 
% 
% %x_all = 0:0.1:25;
% %factor = normpdf(mean_image_all,mean_image_all,std_all);
% %normal_all = normpdf(x_all,mean_image_all,std_all);
% %norm_all = figure('Name','Normal distribution all');
% %plot(x_all,normal_all);
% 
% %x_zero = 75:0.1:125;
% %normal_no_zero = normpdf(x_zero,mean_no_zero,std_no_zero);
% %norm_zero = figure('Name','Normal distribution no zero');
% %plot(x_zero,normal_no_zero);
% 
% 
% disp_ready_ada = disp_ready;
% disp_ready_ada(disp_ready > mean_no_zero) = 0;
% 
% 
% % Plot Stixel ready Disparity
% fig5 = figure('Name','Stixel adapted');
% surf(disp_ready_ada,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
% hold on 
% image(disp_ready_ada,'CDataMapping','scaled')

%% Create Stixels

test_image = zeros(900,1570);

stixel_width = 1;
stixel_matrix = zeros(1570/stixel_width,4); % base_col, start_row, end_row, depth;
stixel_number = 1;
start_row = zeros(1,stixel_width);
end_row = zeros(1,stixel_width);
width = 1;

for col = 1: size(test_image,2)
    non_zero_flag = 0;
    
    % Search for Start row
    row = 900;
    while(non_zero_flag == 0 && row >=1)
        if(disp_ready(row,col)>0)
            non_zero_flag = 1;
            start_row(1,width) = row;
            width = width +1;
            
        end
        row = row - 1;  
    end
    
    if(width == stixel_width +1)
        stixel_matrix(stixel_number,2) = round(mean(start_row(:)));
        width = 1;
        stixel_number = stixel_number +1;
    end
    
    
end



% delta = zeros(1,col);
% stixel_number = 1;
% for col = 1:size(test_image,2)
%     non_detection = 0;
%     start_row = stixel_matrix(stixel_number,2);
%     row = start_row;
%     while(non_detection == 0 && row >=1)
%         delta(1,col) = abs(disp_ready(row,col)-disp_ready(start_row,col));
%         
%         if(delta(1,col)>10)
%             non_detection = 1;
%             end_row(1,width) = row;
%             width = width +1;
%             
%         end
%         row = row - 1;
%     end
%     if(width == stixel_width +1 && stixel_number <= 900/stixel_width )
%         stixel_matrix(stixel_number,3) = mean(end_row(:));
%         width = 1;
%         stixel_number = stixel_number +1;
%     end
%     
%     
% end
% stixel_number = 1;
% width = 1;
% for col = 1: size(test_image,2)
%     non_detection = 0;
%     
%     
%     row = 1;
%     while(non_detection == 0 && row <=900)
%         if(disp_ready(row,col)>0)
%             non_detection = 1;
%             end_row(1,width) = row;
%             width = width +1;
%             
%         end
%         row = row + 1;
%     end
%     
%     if(width == stixel_width +1)
%         stixel_matrix(stixel_number,3) = round(mean(end_row(:)));
%         width = 1;
%         stixel_number = stixel_number +1;
%     end
%     
%     
% end


stixel_vector_full = imresize(stixel_matrix, [1571 5], 'nearest');


fig4 = figure('Name','Stixel ready');
surf(disp_ready,'EdgeColor','none','LineStyle','none','FaceLighting','phong');
hold on 
image(disp_ready,'CDataMapping','scaled');
plot(stixel_vector_full(end:-1:1,2))
%plot(stixel_vector_full(:,3))1













 