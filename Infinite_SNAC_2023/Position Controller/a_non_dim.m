function out = a_non_dim(x_bar, max_states, max_controls, dt)
    
x1_max = max_states(1);
x2_max = max_states(2);
x3_max = max_states(3);
x4_max = max_states(4);
x5_max = max_states(5);
x6_max = max_states(6);

out =     [
    1, 0, 0, (dt*x4_max)/x1_max,  0,  0; 
    0, 1, 0,  0, (dt*x5_max)/x2_max,  0; 
    0, 0, 1,  0,  0, (dt*x6_max)/x3_max; 
    0, 0, 0,  1,  0,  0; 
    0, 0, 0,  0,  1,  0; 
    0, 0, 0,  0,  0,  1; 
    ];

end