function A_bar = A_non_dim(x_bar, dt, Ix, Iy, Iz, x1_max , x2_max, x3_max, x4_max, x5_max, x6_max)

    A_11 = 1 - (dt*(x_bar(6)*x6_max*sin(x_bar(1))*tan(x_bar(2)) - x_bar(5)*x1_max*x4_max*cos(x_bar(1)*x1_max)*tan(x_bar(2)*x2_max)))/x1_max;
    A_12 = (dt*(x_bar(6)*x6_max*cos(x_bar(1))*(tan(x_bar(2))^2 + 1) + x_bar(5)*x2_max*x4_max*sin(x_bar(1)*x1_max)*(tan(x_bar(2)*x2_max)^2 + 1)))/x1_max;
    A_13 = 0;
    A_14 = (dt*x4_max)/x1_max;
    A_15 = (dt*x4_max*sin(x_bar(1)*x1_max)*tan(x_bar(2)*x2_max))/x1_max;
    A_16 = (dt*x6_max*cos(x_bar(1))*tan(x_bar(2)))/x1_max;

    A_21 = -(dt*(x_bar(6)*x1_max*x6_max*cos(x_bar(1)*x1_max) + x_bar(5)*x1_max*x5_max*sin(x_bar(1)*x1_max)))/x2_max;
    A_22 = 1;
    A_23 = 0;
    A_24 = 0;
    A_25 = (dt*x5_max*cos(x_bar(1)*x1_max))/x2_max;
    A_26 = -(dt*x6_max*sin(x_bar(1)*x1_max))/x2_max;
    
    A_31 = (dt*((x_bar(5)*x1_max*x5_max*cos(x_bar(1)*x1_max))/cos(x_bar(2)*x2_max) - (x_bar(6)*x1_max*x6_max*sin(x_bar(1)*x1_max))/cos(x_bar(2)*x2_max)))/x3_max;
    A_32 = (dt*((x_bar(6)*x2_max*x6_max*cos(x_bar(1)*x1_max)*sin(x_bar(2)*x2_max))/cos(x_bar(2)*x2_max)^2 + (x_bar(5)*x2_max*x5_max*sin(x_bar(1)*x1_max)*sin(x_bar(2)*x2_max))/cos(x_bar(2)*x2_max)^2))/x3_max;
    A_33 = 1;
    A_34 = 0;
    A_35 = (dt*x5_max*sin(x_bar(1)*x1_max))/(x3_max*cos(x_bar(2)*x2_max));
    A_36 = (dt*x6_max*cos(x_bar(1)*x1_max))/(x3_max*cos(x_bar(2)*x2_max));

    A_41 = 0;
    A_42 = 0;
    A_43 = 0;
    A_44 = 1; %%wtf
    A_45 = (dt*x_bar(6)*x5_max*x6_max*(Iy - Iz))/(Ix*x4_max);
    A_46 = (dt*x_bar(5)*x5_max*x6_max*(Iy - Iz))/(Ix*x4_max);

    A_51 = 0;
    A_52 = 0;
    A_53 = 0;
    A_54 = -(dt*x_bar(6)*x4_max*x6_max*(Ix - Iz))/(Iy*x5_max);
    A_55 = 1;
    A_56 = -(dt*x_bar(4)*x4_max*x6_max*(Ix - Iz))/(Iy*x5_max);

    A_61 = 0;
    A_62 = 0;
    A_63 = 0;
    A_64 = (dt*x_bar(5)*x4_max*x5_max*(Ix - Iy))/(Iz*x6_max);
    A_65 = (dt*x_bar(4)*x4_max*x5_max*(Ix - Iy))/(Iz*x6_max);
    A_66 = 1;

    A_bar = [
            A_11, A_12, A_13, A_14, A_15, A_16;
            A_21, A_22, A_23, A_24, A_25, A_26;
            A_31, A_32, A_33, A_34, A_35, A_36;
            A_41, A_42, A_43, A_44, A_45, A_46;
            A_51, A_52, A_53, A_54, A_55, A_56;
            A_61, A_62, A_63, A_64, A_65, A_66;
    ];


end