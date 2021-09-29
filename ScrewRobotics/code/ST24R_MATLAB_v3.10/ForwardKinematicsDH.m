%% FORWARDKINEMATICSDH Forwark Kinematics for a Rigid Body Tree by DH
%
% Forward kinematics compute end-effector motion (position & orientation)
% in terms of given joints motion defined by its DH parameters.
%
%   H = FORWARDKINEMATICSDH(DHPARAMS) gets the homogeneous matrix H for 
%   the end-effector of the robot (or rigid body tree of n links)
%   from the Denavit-Hartenberg parameters of each link.
%   dhparams = [d1 t1 r1 a1; d2 t2 r2 a2;...;dn tn rn an] (nx4)
%   for each rigid body (link 1..n).
%       d: The translation on Z(i-1) axis (length units).
%       t: The rotation about Z(i-1) axis (radians). 
%       r: The length on Xi axis of the common normal (joint offset) in 
%          (length units).
%       a: The angle on Xi axis between successive Z(i-1) & Zi axes 
%          (joint twist) in (radians).
%   Only d and t are joint variables.
%
%   Example:
%       %Calculate the Homogeneous Matrix transformation for the
%       %end-effector of a robot with four links whose DH parameters are:
%       dhparams = [1 pi 0 0; 2 pi/2 0 pi/2; 3 0 0 0; 4 pi/4 0 0];
%       %Hst = [-S1C4  S1S4 C1 C1(d3+d4);
%       %        C1C4 -C1S4 S1 S1(d3+d4);
%       %          S4   C4   0     d2+d1;
%       %           0    0   0         1].
%       Hst = ForwardKinematicsDH(dhparams)
%       ans =
%          -0.0000    0.0000   -1.0000   -7.0000
%          -0.7071    0.7071    0.0000    0.0000
%           0.7071    0.7071    0.0000    3.0000
%                0         0         0    1.0000
%
% See also BodyDH2tform(dhparams)
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
% This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB
% 
% ST24R is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published
% by the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ST24R is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Hst = ForwardKinematicsDH(dhparams)
function Hst = ForwardKinematicsDH(dhparams)
    Hst = BodyDH2tform(dhparams(1,:));  
    for i = 2:size(dhparams,1)
        Hst = Hst*BodyDH2tform(dhparams(i,:));
    end
end

