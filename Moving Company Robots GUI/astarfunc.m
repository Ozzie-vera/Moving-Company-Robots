function OptimalPath=astarfunc(StartX,StartY,MAP,GoalRegister,maxsize)


%FINDING ASTAR PATH IN AN OCCUPANCY GRID


% Preallocation of Matrices

[Height,Width]=size(MAP);               %Height and width of matrix

GScore=zeros(Height,Width);           %Matrix keeping track of G-scores 

FScore=single(inf(Height,Width));     %Matrix keeping track of F-scores (only open list) 

Hn=single(zeros(Height,Width));       %Heuristic matrix

OpenMAT=int8(zeros(Height,Width));    %Matrix keeping of open grid cells

ClosedMAT=int8(zeros(Height,Width));  %Matrix keeping track of closed grid cells

ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix

ParentX=int16(zeros(Height,Width));   %Matrix keeping track of X position of parent

ParentY=int16(zeros(Height,Width));   %Matrix keeping track of Y position of parent


%Setting up neighbors to be investigated
Neighboors=[
   -1   ,  0;
    1   ,  0;
    0   , -1;
    0   ,  1; 
            ];
        
Neighboors = Neighboors*maxsize;
N_Neighboors=4; %number of checks
%%% End of setting up matrices representing neighboors to be investigated


%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
[col, row]=find(GoalRegister==1); %finds col and row of goal point
RegisteredGoals=[row col];
Nodesfound=size(RegisteredGoals,1);

for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        if MAP(k,j)==0
            Mat=RegisteredGoals-(repmat([j k],(Nodesfound),1));
            Distance=(min(sqrt(sum(abs(Mat).^2,2))));
            Hn(k,j)=Distance;
        end
    end
end
%End of creating Heuristic-matrix. 

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartY,StartX)=Hn(StartY,StartX);   %Start f score at starting point      
OpenMAT(StartY,StartX)=1;   




while 1==1 %Code will break when path found or when no path exist
    MINopenFSCORE=min(min(FScore));
    
    if MINopenFSCORE==inf%check for failure
    
    OptimalPath=inf;
    RECONSTRUCTPATH=0;
     break
    end
    
    
    [CurrentY,CurrentX]=find(FScore==MINopenFSCORE);
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);

    if GoalRegister(CurrentY,CurrentX)==1
    %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
  %Removing node from OpenList to ClosedList  
    OpenMAT(CurrentY,CurrentX)=0;
    FScore(CurrentY,CurrentX)=inf;
    ClosedMAT(CurrentY,CurrentX)=1;
    
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        %check for the boundaries
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width
            continue
        end
        Flag=1; 
        if(ClosedMAT(CurrentY+i,CurrentX+j)==0) %Neiboor is open;
            if (abs(i)>1||abs(j)>1) %goes from 1- i or j whichever is above 1
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);
            
                    if (MAP(CurrentY+YPOS,CurrentX+XPOS)==1)
                        Flag=0;
                    end
                        
                    if GoalRegister(CurrentY+YPOS,CurrentX+XPOS)==1
                        %GOAL!!
                        RECONSTRUCTPATH=1;
                        break
                    end
                end
            end
             %End of  checking that the path does not pass an object

            if Flag==1          
                tentative_gScore = GScore(CurrentY,CurrentX) + sqrt(i^2+j^2);
                if OpenMAT(CurrentY+i,CurrentX+j)==0
                    OpenMAT(CurrentY+i,CurrentX+j)=1;                    
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j)=CurrentX;
                ParentY(CurrentY+i,CurrentX+j)=CurrentY;
                GScore(CurrentY+i,CurrentX+j)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j)= tentative_gScore+Hn(CurrentY+i,CurrentX+j);
            end
        end
    end
    
    
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX];
    while RECONSTRUCTPATH
        
        PrevXTemp=ParentX(CurrentY,CurrentX);%put previous x in xTemp
        PrevYTemp=ParentY(CurrentY,CurrentX); %put previous y in yTemp

        CurrentY=PrevYTemp;
        CurrentX=PrevXTemp;
        OptimalPath(k,:)=[CurrentY CurrentX];
        k=k+1;
        
        if (((CurrentX== StartX)) &&(CurrentY==StartY))
            break
        end
    end
end


end

      
    

