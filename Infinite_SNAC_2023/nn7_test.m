

ux =10;
uy=-2;
uz=-.5;

    [ft_angles] = NN7(ux, uy, uz);
    ft = ft_angles(1);

    angles(:) = [ft_angles(2);ft_angles(3); 0];