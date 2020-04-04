%Example on the use of AStar Algorithm in an occupancy grid. 
close all
clc


%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
MAP = xlsread('map.xlsx');

%Start Positions
StartX=50;
StartY= 99;

%Generating goal nodes, which is represented by a matrix. Several goals can be speciefied, in which case the pathfinder will find the closest goal. 
%a cell with the value 1 represent a goal cell
%GoalRegister=xlsread('Goal.xlsx');
destx = 16;
desty = 40;
GoalRegister=int8(zeros(128,140));
GoalRegister(desty-3,destx-3:destx+3) = 1;
GoalRegister(desty-2,destx-3:destx+3) = 1;
GoalRegister(desty-1,destx-3:destx+3) = 1;
GoalRegister(desty,destx-3:destx+3) = 1;
GoalRegister(desty+1,destx-3:destx+3) = 1;
GoalRegister(desty+2,destx-3:destx+3) = 1;
GoalRegister(desty+3,destx-3:destx+3) = 1;
GoalRegister(110,80)=1;



% Running PathFinder
OptimalPath=astarfunc(StartX,StartY,MAP,GoalRegister,8);
% End. 

if size(OptimalPath,2)>1
figure(10)
imagesc((MAP));
colormap(flipud(gray));

hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'x','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')


else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end



% %1
% NeigboorCheck=[0 1 0;1 0 1;0 1 0];
% figure(1)
% [row col]=find(NeigboorCheck==1);
% Neighboors=[row col]-(1+1);
% 
% for p=1:size(Neighboors,1)
%   i=Neighboors(p,1);
%        j=Neighboors(p,2);
%       
%      plot([0 i],[0 j],'k')
%  hold on
%  axis equal
%  grid on
% 
% 
% end



