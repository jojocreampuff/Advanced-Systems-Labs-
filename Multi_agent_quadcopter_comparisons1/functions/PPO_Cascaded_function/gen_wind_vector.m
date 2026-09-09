function Wxyz = gen_wind_vector(W_nominal, dt, tf)
    % rng(5,"twister")
    T = 0:dt:tf-dt;
    N = length(T);
    % wind modeling
    W = zeros(1,N);
    
    W(1) = W_nominal; % nominal wind speed
    heading_nominal = pi/2;
    heading = zeros(1,N);
    incline = zeros(1,N);
    incline(1) = 0;
    heading(1) = heading_nominal; % wind headng east
    std_div_wind = 5; % standard divation of wind mag is 1m/s
    std_div_wind_heading = deg2rad(30); % 10 degrees std of heading
    % std_div_wind_incline = deg2rad(0);
    Tw = 20; % drift time 
    T_heading = 20;
    % T_incline = 20;
    
    for i = 1:N
        % first order wind model
    
        % wind mag dynamics: drift around nominal value
        W_dot = -((W(i)-W(1))/Tw) + std_div_wind*randn;
        W(i+1) = W(i) + dt*W_dot;
        % wind direction dynamics: drift around 0 degrees
        % incline_dot = -(incline(i)/T_incline) + std_div_wind_incline*randn;
        % incline(i+1) = incline(i) + dt*incline_dot;
        heading_dot = -(heading(i)/T_heading) + std_div_wind_heading*randn;
        heading(i+1) = heading(i) + dt*heading_dot;
        % NED global wind componates: find glo bal x, y, z wind speeds 
        Wxyz(:,i) = [W(i)*cos(incline(i))*cos(heading(i));
            W(i)*cos(incline(i))*sin(heading(i));
            -W(i)*sin(incline(i))];
        
    end

