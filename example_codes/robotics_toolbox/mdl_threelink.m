%MDL_THREELINK Create model of a simple 3-link mechanism
%
%      mdl_threelink
%
% Script creates the workspace variable tl which describes the 
% kinematic and dynamic characteristics of a simple planar 3-link mechanism.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
%
% References::
%  - Based on 3 link simulated robot in our IJRR paper
%

a1 = 0.196;
a2 = 0.334;
a3 = 0.288;
%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
L(3) = Link([ 0     0   a3  0], 'standard');


threelink = SerialLink(L, 'name', 'three link', ...
		       'comment', 'from Spong, Hutchinson, Vidyasagar');
qz = [0 0 0];
qn = [pi/6, -pi/6, pi/4];
