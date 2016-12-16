%MDL_CODY Create model of Cody manipulator from Meka
%
%      mdl_cody_no_motors
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

clear L

%%%%%%%%%%%%%%%%this set of parameters was one that I got by drawing the
%%%%%%%%%%%%%%%%frames in slightly different way, both this and the one
%%%%%%%%%%%%%%%%below that we came up with in class are valid!
%%%%%%%%
%%%%%%%%%%%%% order of parameters is the following:
%%%%%%%%%%%%% theta, d, a, alpha, revolute or prismatic, offset
% L(1) = Link([ 0     0.18493 0         pi/2    0         pi/2], 'standard');
% L(2) = Link([ 0 	0.      0.03175   pi/2    0         pi/2], 'standard');
% L(3) = Link([ 0     0.27795	0        -pi/2    0        -pi/2], 'standard');
% L(4) = Link([ 0     0       0         pi/2    0         0   ], 'standard');
% L(5) = Link([ 0     0.27853 0        -pi/2    0         0   ], 'standard');
% L(6) = Link([ 0     0       0        -pi/2    0         pi/2], 'standard');
% L(7) = Link([ 0     0       -0.1      0       0         0   ], 'standard');


%%%% these are the DH parameters that we came up with
%%%% in class to describe the Forward Kinematics
L(1) = Link([ 0     0.18493 0          pi/2     0       0], 'standard');
L(2) = Link([ 0 	0.      0.03175   -pi/2     0       pi/2], 'standard');
L(3) = Link([ 0     0.27795	0          pi/2     0       pi/2], 'standard');
L(4) = Link([ 0     0       0          pi/2     0       0   ], 'standard');
L(5) = Link([ 0     -0.27853 0         -pi/2     0       0   ], 'standard');
L(6) = Link([ 0     0       0          pi/2     0       pi/2], 'standard');
L(7) = Link([ 0     0       0.1         0      0       0   ], 'standard');

%% defining the robot now
cody = SerialLink(L, 'name', 'Cody', ...
    'manufacturer', 'Meka', 'comment', 'params from Meka');

%%this allows use to line up the first or base frame in a way we want, it
%%is a static transformation that gets added to all of our FK calculations
cody.base = [0 1 0 0;
             0 0 1 0;
             1 0 0 0;
             0 0 0 1];

% some useful poses
qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose

clear L
