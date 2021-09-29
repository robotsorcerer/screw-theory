%% FORWARDKINEMATICSPOE Forwark Kinematics for a Rigid Body Tree by POE
% Product Of Exponentials from a Screw Theory movement.
% Use in SE(3).
%
% HstR = ForwardKinematicsPOE(TwMag)
%
% Forward kinematics compute the RELATIVE TO THE STATIONARY SYSTEM
% end-effector motion (pos & rot) into a homogeneous matrix tform(4x4)
% as a funtion of the given joints motion defined by the "Twist-Mangitude"
% of n links.
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (link 1..n).
% Twn1..Twn6: The TWIST components for the joint SCREW movement.
% Magn: The MAGNITUDE component for the joint SCREW movement.
%
%   Example:
%       %Calculate the Homogeneous Matrix RELATIVE transformation for the
%       %end-effector of a robot with four links whose "Twist-Mangitude"
%       %parameters are:
%       TwMag1 = [xi1 t1]' = [[0 0 0 0 0 1] pi]';
%       TwMag2 = [xi2 t2]' = [[0 0 1 0 0 0] 2]';
%       TwMag3 = [xi3 t3]' = [[1 0 0 0 0 0] 3]';
%       TwMag4 = [xi4 t4]' = [[0 1 0 1 0 0] pi/4]';
%       TwMag = [TwMag1 TwMag2 TwMag3 TwMag4];
%       HstR = ForwardKinematicsPOE(TwMag)
%       ans =
%       -1.0000   -0.0000    0.0000   -3.0000
%        0.0000   -0.7071    0.7071   -0.7071
%             0    0.7071    0.7071    2.2929
%             0         0         0    1.0000
%
% See also ForwardKinematicsDH(dhparams)
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
%% HstR = ForwardKinematicsPOE(TwMag)
function HstR = ForwardKinematicsPOE(TwMag)
    HstR = expScrew(TwMag(:,1));  
    for i = 2:size(TwMag,2)
        HstR = HstR*expScrew(TwMag(:,i));
    end
end