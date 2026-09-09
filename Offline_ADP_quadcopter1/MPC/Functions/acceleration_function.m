function acceleration=acceleration_function(controls)
%Controls is [thrust;roll;pitch;yaw]
thrust=controls(1);
roll=controls(2);
pitch=controls(3);
yaw=controls(4);
ax=-(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))*thrust/1;
ay=-(cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))*thrust/1;
az=-cos(roll)*cos(pitch)*thrust/1+9.81;
acceleration=[ax;ay;az];