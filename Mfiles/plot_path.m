% This mfile plots the generated paths by RRT* algorithm.
% It reads the .TXT files that C++ code generates
 

clear
clc
%% The width and height of the world need to be manually set
WORLD_WIDTH = 500.0;
WORLD_HEIGHT = 500.0;

%% Plot the boundaries of the world!
 pgon = polyshape([0 0 WORLD_WIDTH WORLD_WIDTH],[WORLD_HEIGHT 0 0 WORLD_HEIGHT]);
 plot(pgon,'FaceAlpha',0.)
hold on;
%% Reads Obstacles.txt file and plot the obstacles.
filename = 'Obstacles.txt';
delimiterIn = '\t';
headerlinesIn =2 ;
obs = importdata(filename,delimiterIn,headerlinesIn);


for i=1:1: size(obs.data,1)
    ob_1=[obs.data(i,1),obs.data(i,2)];
    ob_2=[obs.data(i,3),obs.data(i,4)];
    pgon = polyshape([ob_1(1) ob_1(1) ob_2(1) ob_2(1)],[ob_1(2) ob_2(2) ob_2(2) ob_1(2)]);
    p3=plot(pgon);
end


%% Reads first_viable_path.txt file and plot the first viable path that RRT* has generated
filename = 'first_viable_path.txt';
delimiterIn = '\t';
headerlinesIn =2 ;
Path1 = importdata(filename,delimiterIn,headerlinesIn);
if isfield(Path1,'data')
    p1=plot(Path1.data(:,1),Path1.data(:,2),'b*-');
end

%% Reads Path_after_MAX_ITER.txt file and plot the optimized path after maximum iteration
filename = 'Path_after_MAX_ITER.txt';
delimiterIn = '\t';
headerlinesIn =2 ;
Path2 = importdata(filename,delimiterIn,headerlinesIn);
    
if isfield(Path2,'data')
    p2=plot(Path2.data(:,1),Path2.data(:,2),'rs--');
end


%% Set the legends for the plot.
if exist('p1','var') && exist('p2','var')
    legend([p1 p2],{'First viable path','Path after MAX\_ITER'},'Location','best')
elseif exist('p1','var')&& ~exist('p2','var')
    legend(p1,{'First viable path'},'Location','best')
elseif ~exist('p1','var')&& exist('p2','var')
    legend(p2,{'Path after MAX\_ITER'},'Location','best')
end
