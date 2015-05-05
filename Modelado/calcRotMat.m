%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INPUT -> The array with the three ptp that define the orientation  %%
%%          of the quad.                                                 %%
%% OUTPUTS -> The rotation matrix result of three rotations according to %%
%%            the ptp provided.                                       %%
%%         -> Three matrices that describe each individual turn.         %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [LHb, LH1, L12, L2b] = calcRotMat(ptp)
 
    LH1 = [cos(ptp(3))   sin(ptp(3))  0;
           -sin(ptp(3))  cos(ptp(3))  0;
                 0               0         1];
             
             
    L12 = [cos(ptp(2))  0  -sin(ptp(2));
                 0         1        0       ;
           sin(ptp(2))  0   cos(ptp(2))];
       
       
    L2b = [1          0              0        ;
           0     cos(ptp(1)) sin(ptp(1));
           0    -sin(ptp(1)) cos(ptp(1))];
    
    LHb = LH1 * L12 * L2b;
    
end