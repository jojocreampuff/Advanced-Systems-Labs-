function [reference_x,reference_y,reference_z]=Reference_Generation(Line_reference,Step_reference,Sin_reference,time,dt)
% Creates a position reference of (3 x time) size


% Straight Line Reference
if Line_reference==1
    reference_x = linspace(0, 10, time/dt);
    reference_y = linspace(0, 2, time/dt);
    reference_z = -linspace(0, 20, time/dt);
end
% Step Reference
if Step_reference==1
    % Parameters
    reference_x=zeros(1,time/dt);
    reference_y=zeros(1,time/dt);
    reference_z=zeros(1,time/dt);
    for i=time/dt-100-4:time/dt-100-1
        reference_x(i)=-(time/dt-100-4)*.25+i*.25;
    end
    for i=time/dt-100:time/dt
        reference_x(i)=1;

    end
    for i=time/dt-200-4:time/dt-200-1
        reference_y(i)=-(time/dt-200-4)*.25+i*.25;
    end
    for i=time/dt-200:time/dt
        reference_y(i)=1;
    end
    for i=time/dt/5-4:time/dt/5-1
        reference_z(i)=-(-(time/dt/5-4)*.25+i*.25);
    end
    for i=time/dt/5:time/dt
        reference_z(i)=-(1);
    end
end
% Sin reference
if Sin_reference==1
    reference_x = 3*sin(linspace(0, 2 * pi, time/dt));
    reference_y = 3*sin(linspace(0, 2 * pi, time/dt)+pi/6);
    reference_z = 4*sin(linspace(0, 2 * pi, time/dt));
end
