% This mfile plots the generated paths by RRT* algorithm.
% It reads the .TXT files that C++ code generates
 

clear
clc

%% Some parameters to set (Filename, output PNG name)

% YOU NEED TO MODIFY!
version = '15'
GIF_SPACE = 5; % .gif 間隔多少ii要get_grame()一次

start_x = 425;
start_y = 475;
goal_x = 25;
goal_y = 150;

% INPUT Files
OBSTACLE_FILE = 'Obstacles_c4.txt';
FIRST_PATH = strcat('FirstPath/first_path_v', version, '.txt');
OPTIMIZE_PATH = strcat('OptPath/opt_path_v' ,version ,'.txt');
AVAILABLE_POINTS_FILE = strcat('AvailablePts/avail_pts_v', version,'.txt');
NODES_POINTS_FILE = strcat('Nodes/nodes_pts_v', version, '.txt');

% OUTPUT Image
PNG_NAME = strcat('Graphs/graph_v',version,'.png');

%% The width and height of the world need to be manually set
WORLD_WIDTH = 500.0;
WORLD_HEIGHT = 500.0;

%% Plot the boundaries of the world!

h = figure;
pgon = polyshape([0 0 WORLD_WIDTH WORLD_WIDTH],[WORLD_HEIGHT 0 0 WORLD_HEIGHT]);
 plot(pgon,'FaceAlpha',0.)
hold on;

%% Plot starting & goal location
start = [start_x, start_y]
goal = [goal_x, goal_y]
plot(start_x, start_y,'rs');
plot(goal_x, goal_y,'go');

%% Reads Obstacles.txt file and plot the obstacles.
%filename = 'Obstacles.txt';
filename = OBSTACLE_FILE;
delimiterIn = '\t';
headerlinesIn =2 ;
obs = importdata(filename,delimiterIn,headerlinesIn);


for i=1:1: size(obs.data,1)
    ob_1=[obs.data(i,1),obs.data(i,2)];
    ob_2=[obs.data(i,3),obs.data(i,4)];
    pgon = polyshape([ob_1(1) ob_1(1) ob_2(1) ob_2(1)],[ob_1(2) ob_2(2) ob_2(2) ob_1(2)]);
    p3=plot(pgon);
end


%% Reads available point available_c4.txt file and plot reachable workspace
%filename = 'Path_after_MAX_ITER.txt';
%filename = AVAILABLE_POINTS_FILE;
%delimiterIn = '\t';
%headerlinesIn =2 ;
%avail_pts = importdata(filename,delimiterIn,headerlinesIn);
    
%if isfield(avail_pts,'data')
%    p4=plot(avail_pts.data(:,1),avail_pts.data(:,2),'g*'); %g--o
%end

%% Reads first_viable_path.txt file and plot the first viable path that RRT* has generated
%filename = 'first_viable_path.txt';
filename = FIRST_PATH;
delimiterIn = '\t';
headerlinesIn =2 ;
Path1 = importdata(filename,delimiterIn,headerlinesIn);
%if isfield(Path1,'data')
%    p1=plot(Path1.data(:,1),Path1.data(:,2),'r+--', 'linewidth', 1.5);
%end

%% Reads Path_after_MAX_ITER.txt file and plot the optimized path after maximum iteration
%filename = 'Path_after_MAX_ITER.txt';
filename = OPTIMIZE_PATH;
delimiterIn = '\t';
headerlinesIn =2 ;
Path2 = importdata(filename,delimiterIn,headerlinesIn);
    
%if isfield(Path2,'data')
%    p2=plot(Path2.data(:,1),Path2.data(:,2),'bs-', 'linewidth', 1.5);
%end



%% Reads nodes point .txt file and plot reachable workspace
%filename = 'Path_after_MAX_ITER.txt';
filename = NODES_POINTS_FILE;
delimiterIn = '\t';
headerlinesIn =2 ;
nodes_pts = importdata(filename,delimiterIn,headerlinesIn);
    
%if isfield(nodes_pts,'data')
%    p5=plot(nodes_pts.data(:,1),nodes_pts.data(:,2),'b*'); %g--o
%end

[edgesRowCount,~] = size(nodes_pts.data)
  
vertices = nodes_pts.data;

%% Plot RRT .gif
%h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
GIF_NAME = strcat('Graphs/graph_v',version,'.gif');
filename = GIF_NAME;
%edgesRowCount
for ii = 1 : edgesRowCount
  plot(vertices(ii, 1), vertices(ii, 2), 'g--*', 'linewidth', 1);
  drawnow 
      % Capture the plot as an image 
%      frame = getframe(h); 
%      im = frame2im(frame); 
%      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if ii == 1 
          frame = getframe(h); 
          im = frame2im(frame); 
          [imind,cm] = rgb2ind(im,256); 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      elseif mod(ii,GIF_SPACE)==0
          frame = getframe(h); 
          im = frame2im(frame); 
          [imind,cm] = rgb2ind(im,256);             
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          X = sprintf('frame %d drawn',ii);
          disp(X)
      elseif ii == edgesRowCount-1
       
        if isfield(Path1,'data')
            p1=plot(Path1.data(:,1),Path1.data(:,2),'r+--', 'linewidth', 1.5);
        end          
         if isfield(Path2,'data')
            p2=plot(Path2.data(:,1),Path2.data(:,2),'bs-', 'linewidth', 1.5);
        
         end    
            % Set the legends for the plot.
            if exist('p1','var') && exist('p2','var')
                legend([p1 p2],{'First viable path','Path after MAX\_ITER'},'Location','best')
                %legend([p1,p2,p3],{'First viable path','Path after MAX\_ITER','Reachable points after RRT* exploration'},'Location','best')
            elseif exist('p1','var')&& ~exist('p2','var')
                legend(p1,{'First viable path'},'Location','best')
            elseif ~exist('p1','var')&& exist('p2','var')
                legend(p2,{'Path after MAX\_ITER'},'Location','best')
            end         
       
          frame = getframe(h); 
          im = frame2im(frame); 
          [imind,cm] = rgb2ind(im,256);             
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          X = sprintf('Done plotting first and optimized path.');
          disp(X)        
      end 
end

%legend(p4,{'Reachable points after RRT exploration'},'Location','best')

%legend(p5,{'nodes points after RRT exploration'},'Location','best')


%% Save output plot to PNG image file.

saveas(h,PNG_NAME);

disp('Done saving the image.');
