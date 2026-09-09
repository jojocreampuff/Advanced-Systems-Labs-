function movieVector = drone_Animation(x,y,z,roll,pitch,yaw,r_x, r_y, r_z)
% This Animation code is for QuadCopter. Written by Jitendra Singh 

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
  sin(ro) cos(ro)  0;
   0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
       -a/2 -a/2 a/2 a/2;
         0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
fig1 = figure('pos', [0 50 1080 720]);
% set(fig1, 'Renderer', 'zbuffer');
hg   = gca;
% grid on;
axis equal;
% xlim([-3.2 3.2]); ylim([-3.2 3.2]); zlim([0 3]);
xlim([-6 6]); ylim([-6 6]); zlim([0 4]);
% title('Safe Quadcopter Tracking','FontSize', 14, 'Interpreter', 'latex')
xlabel('X (m)' ,'FontSize', 14, 'Interpreter', 'latex');
ylabel('Y (m)','FontSize', 14, 'Interpreter', 'latex');
zlabel('Z (m)','FontSize', 14, 'Interpreter', 'latex');
view(-130, -35); % Sets azimuth to 45 degrees and elevation to 30 degrees

hold(gca, 'on');

%% Design Different parts
% design the base square
drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
[xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
alpha(drone(5:8),0.7);
% design 4 propellers
drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
alpha(drone(9:12),0.3);

%% create a group object and parent surface
combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)

ref = plot3(r_x,r_y,-r_z, 'g--','LineWidth',1.5); %
%% barrier information
radius = 2;
c = [-1;-1.5;-2];
nSphere = 30;                              % resolution
[Xs,Ys,Zs] = sphere(nSphere);              % unit sphere
Xs = radius*Xs + c(1);
Ys = radius*Ys + c(2);
Zs = radius*Zs + c(3);


 for i = 1:length(x)
     
     ba = plot3(x(1:i),y(1:i),z(1:i), 'k:','LineWidth',2);
    surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');

     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',translation*rotation3*rotation2*rotation1);
      movieVector(i) =  getframe(fig1);
     drawnow
 end

 
