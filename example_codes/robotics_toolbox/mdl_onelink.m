%MDL_ONELINK Create model of a 1-link mechanism
%
a1 = 1;
%   theta d a alpha

onelink = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'm', 1, 'r', [-0.5 0 0], 'I', [0 0 0.5], 'B', 0.1 , 'G', 1, 'Jm', 0, 'standard')
    ], ...
    'name', 'one link');
