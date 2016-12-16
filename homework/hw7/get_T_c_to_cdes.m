function T_c_to_cdes = get_T_c_to_cdes(u)
    xyz = transl(u)
    rpy = tr2rpy(u,'deg')
    T_cdes_to_p = [0 0 -1 0;...
                   0 1 0 0;...
                   1 0 0 -0.5;...
                   0 0 0 1];
    
%     T_cdes_to_p = u
    lambda = 0.2;
    T_c_to_cdes = lambda*(u*inv(T_cdes_to_p))
    [x y z] = transl(T_c_to_cdes);
    rpy = tr2rpy(T_c_to_cdes);
    
end