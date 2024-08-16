function A_bar = A_full_non_dim(x_bar, dt, m, Ix, Iy, Iz, max_states, Ft_max, Ft)

% Ft is Ft_bar 
x1_max = max_states(1);
x2_max = max_states(2);
x3_max = max_states(3);
x4_max = max_states(4);
x5_max = max_states(5);
x6_max = max_states(6);
x7_max = max_states(7);
x8_max = max_states(8);
x9_max = max_states(9);
x10_max = max_states(10);
x11_max = max_states(11);
x12_max = max_states(12);

% assume these are all x_bar!!
x1 = x_bar(1);      x2 = x_bar(2);      x3 = x_bar(3);
x4 = x_bar(4);      x5 = x_bar(5);      x6 = x_bar(6);
x7 = x_bar(7);      x8 = x_bar(8);      x9 = x_bar(9);
x10 = x_bar(10);    x11 = x_bar(11);    x12 = x_bar(12);

A_11 = 1;
A_12 = 0;
A_13 = 0;
A_14 = (dt * x4_max) / x1_max;
A_15 = 0;
A_16 = 0;
A_17 = 0;
A_18 = 0;
A_19 = 0;
A_110 = 0;
A_1_11 = 0;
A_1_12 = 0;

A_21 = 0;
A_22 = 1;
A_23 = 0;
A_24 = 0;
A_25 = (dt * x5_max) / x2_max;
A_26 = 0;
A_27 = 0;
A_28 = 0;
A_29 = 0;
A_210 = 0;
A_211 = 0;
A_212 = 0;

A_31 = 0;
A_32 = 0;
A_33 = 1;
A_34 = 0;
A_35 = 0;
A_36 = (dt * x6_max) / x3_max;
A_37 = 0;
A_38 = 0;
A_39 = 0;
A_310 = 0;
A_311 = 0;
A_312 = 0;

A_41 = 0;
A_42 = 0;
A_43 = 0;
A_44 = 1;
A_45 = 0;
A_46 = 0;
A_47 = -(Ft_max * dt^2 * conj(Ft) * (x7_max * cos(x7 * x7_max) * sin(x9 * x9_max) - x7_max * cos(x9 * x9_max) * sin(x7 * x7_max) * sin(x8 * x8_max))) / (m * x4_max * conj(Ft_max));
A_48 = -(Ft_max * dt^2 * x8_max * conj(Ft) * cos(x7 * x7_max) * cos(x8 * x8_max) * cos(x9 * x9_max)) / (m * x4_max * conj(Ft_max));
A_49 = -(Ft_max * dt^2 * conj(Ft) * (x9_max * cos(x9 * x9_max) * sin(x7 * x7_max) - x9_max * cos(x7 * x7_max) * sin(x8 * x8_max) * sin(x9 * x9_max))) / (m * x4_max * conj(Ft_max));
A_410 = 0;
A_411 = 0;
A_412 = 0;

A_51 = 0;
A_52 = 0;
A_53 = 0;
A_54 = 0;
A_55 = 1;
A_56 = 0;
A_57 = (Ft_max * dt^2 * conj(Ft) * (x7_max * cos(x7 * x7_max) * cos(x9 * x9_max) + x7_max * sin(x7 * x7_max) * sin(x8 * x8_max) * sin(x9 * x9_max))) / (m * x5_max * conj(Ft_max));
A_58 = -(Ft_max * dt^2 * x8_max * conj(Ft) * cos(x7 * x7_max) * cos(x8 * x8_max) * sin(x9 * x9_max)) / (m * x5_max * conj(Ft_max));
A_59 = -(Ft_max * dt^2 * conj(Ft) * (x9_max * sin(x7 * x7_max) * sin(x9 * x9_max) + x9_max * cos(x7 * x7_max) * cos(x9 * x9_max) * sin(x8 * x8_max))) / (m * x5_max * conj(Ft_max));
A_510 = 0;
A_511 = 0;
A_512 = 0;

A_61 = 0;
A_62 = 0;
A_63 = 0;
A_64 = 0;
A_65 = 0;
A_66 = 1;
A_67 = (Ft_max * dt^2 * x7_max * conj(Ft) * cos(x8 * x8_max) * sin(x7 * x7_max)) / (m * x6_max * conj(Ft_max));
A_68 = (Ft_max * dt^2 * x8_max * conj(Ft) * cos(x7 * x7_max) * sin(x8 * x8_max)) / (m * x6_max * conj(Ft_max));
A_69 = 0;
A_610 = 0;
A_611 = 0;
A_612 = 0;

A_71 = 0;
A_72 = 0;
A_73 = 0;
A_74 = 0;
A_75 = 0;
A_76 = 0;
A_77 = (dt * (x11 * x11_max * x7_max * cos(x7 * x7_max) * tan(x8 * x8_max) - x12 * x12_max * x7_max * sin(x7 * x7_max) * tan(x8 * x8_max))) / x7_max + 1;
A_78 = (dt * (x11 * x11_max * x8_max * sin(x7 * x7_max) * (tan(x8 * x8_max)^2 + 1) + x12 * x12_max * x8_max * cos(x7 * x7_max) * (tan(x8 * x8_max)^2 + 1))) / x7_max;
A_79 = 0;
A_710 = (dt * x10_max) / x7_max;
A_711 = (dt * x11_max * sin(x7 * x7_max) * tan(x8 * x8_max)) / x7_max;
A_712 = (dt * x12_max * cos(x7 * x7_max) * tan(x8 * x8_max)) / x7_max;

A_81 = 0;
A_82 = 0;
A_83 = 0;
A_84 = 0;
A_85 = 0;
A_86 = 0;
A_87 = -(dt * (x12 * x12_max * x7_max * cos(x7 * x7_max) + x11 * x11_max * x7_max * sin(x7 * x7_max))) / x8_max;
A_88 = 1;
A_89 = 0;
A_810 = 0;
A_811 = (dt * x11_max * cos(x7 * x7_max)) / x8_max;
A_812 = -(dt * x12_max * sin(x7 * x7_max)) / x8_max;

A_91 = 0;
A_92 = 0;
A_93 = 0;
A_94 = 0;
A_95 = 0;
A_96 = 0;
A_97 = (dt * ((x11 * x11_max * x7_max * cos(x7 * x7_max)) / cos(x8 * x8_max) - (x12 * x12_max * x7_max * sin(x7 * x7_max)) / cos(x8 * x8_max))) / x9_max;
A_98 = (dt * ((x12 * x12_max * x8_max * cos(x7 * x7_max) * sin(x8 * x8_max)) / cos(x8 * x8_max)^2 + (x11 * x11_max * x8_max * sin(x7 * x7_max) * sin(x8 * x8_max)) / cos(x8 * x8_max)^2)) / x9_max;
A_99 = 1;
A_910 = 0;
A_911 = (dt * x11_max * sin(x7 * x7_max)) / (x9_max * cos(x8 * x8_max));
A_912 = (dt * x12_max * cos(x7 * x7_max)) / (x9_max * cos(x8 * x8_max));

A_101 = 0;
A_102 = 0;
A_103 = 0;
A_104 = 0;
A_105 = 0;
A_106 = 0;
A_107 = 0;
A_108 = 0;
A_109 = 0;
A_1010 = 1;
A_1011 = (dt * x12 * x11_max * x12_max * (Iy - Iz)) / (Ix * x10_max);
A_1012 = (dt * x11 * x11_max * x12_max * (Iy - Iz)) / (Ix * x10_max);

A_111 = 0;
A_112 = 0;
A_113 = 0;
A_114 = 0;
A_115 = 0;
A_116 = 0;
A_117 = 0;
A_118 = 0;
A_119 = 0;
A_1110 = -(dt * x12 * x10_max * x12_max * (Ix - Iz)) / (Iy * x11_max);
A_1111 = 1;
A_1112 = -(dt * x10 * x10_max * x12_max * (Ix - Iz)) / (Iy * x11_max);

A_121 = 0;
A_122 = 0;
A_123 = 0;
A_124 = 0;
A_125 = 0;
A_126 = 0;
A_127 = 0;
A_128 = 0;
A_129 = 0;
A_1210 = (dt * x11 * x10_max * x11_max * (Ix - Iy)) / (Iz * x12_max);
A_1211 = (dt * x10 * x10_max * x11_max * (Ix - Iy)) / (Iz * x12_max);
A_1212 = 1;

% Partial x_k+1 / partial x_k grad(f(x) + g(x) *u)
A_bar = [
    A_11, A_12, A_13, A_14, A_15, A_16, A_17, A_18, A_19, A_110, A_1_11, A_1_12;
    A_21, A_22, A_23, A_24, A_25, A_26, A_27, A_28, A_29, A_210, A_211, A_212;
    A_31, A_32, A_33, A_34, A_35, A_36, A_37, A_38, A_39, A_310, A_311, A_312;
    A_41, A_42, A_43, A_44, A_45, A_46, A_47, A_48, A_49, A_410, A_411, A_412;
    A_51, A_52, A_53, A_54, A_55, A_56, A_57, A_58, A_59, A_510, A_511, A_512;
    A_61, A_62, A_63, A_64, A_65, A_66, A_67, A_68, A_69, A_610, A_611, A_612;
    A_71, A_72, A_73, A_74, A_75, A_76, A_77, A_78, A_79, A_710, A_711, A_712;
    A_81, A_82, A_83, A_84, A_85, A_86, A_87, A_88, A_89, A_810, A_811, A_812;
    A_91, A_92, A_93, A_94, A_95, A_96, A_97, A_98, A_99, A_910, A_911, A_912;
    A_101, A_102, A_103, A_104, A_105, A_106, A_107, A_108, A_109, A_1010, A_1011, A_1012;
    A_111, A_112, A_113, A_114, A_115, A_116, A_117, A_118, A_119, A_1110, A_1111, A_1112;
    A_121, A_122, A_123, A_124, A_125, A_126, A_127, A_128, A_129, A_1210, A_1211, A_1212;
];


end