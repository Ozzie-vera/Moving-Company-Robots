
i = -8;
j = 0;
if (abs(i)>1||abs(j)>1) 
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);
            %MAP(CurrentY+YPOS,CurrentX+XPOS)
                    if (0==1)
                        Flag=0;
                    end
                    
                end
                
end
