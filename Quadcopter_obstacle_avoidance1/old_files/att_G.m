function out = att_G(x)
Ix = 0.3;
Iy = 0.4;
Iz = 0.5;
out = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
end