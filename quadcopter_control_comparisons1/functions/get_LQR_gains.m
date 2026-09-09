function [K_pos, K_att] = get_LQR_gains(Q_pos,R_pos, Q_att, R_att, Ix, Iy, Iz)
    %% Make sure that these are the same as in the snac training files
A_pos = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0   
        0   0   0   0   0   0];

B_pos = [0  0   0  
        0   0   0
        0   0   0
        -1   0   0
        0   -1   0
        0   0   -1];

% att A and B 
% the control from this lqr will be taux tauy tauz
A_att = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0  
        0   0   0   0   0   0];

B_att = [0   0   0 
         0   0   0 
         0   0   0 
         1/Ix    0   0
         0   1/Iy    0
         0    0      1/Iz];

[K_pos,S,CLP] = lqr(A_pos,B_pos,Q_pos,R_pos);
[K_att,S_att,CLP_att] = lqr(A_att,B_att,Q_att,R_att);

    