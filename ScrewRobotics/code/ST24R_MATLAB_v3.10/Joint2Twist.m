%% JOINT2TWIST gets de TWIST from the Joint AXIS and a POINT on that axis
% Use in SE(3).
%
% where the TWIST "xi" (6x1) has the two components v (3x1) and w (3x1)
% From AXIS (3x1) & a POINT (3X1) on that axis AT THE REFERENCE POSITION.
% It is also necessary to indicate the type of JOINTTYPE ('rot' or 'tra')
% for the function to work with both ROTATION & TRANSLATION movements.
% Use in SE(3).
%
%   xi = Joint2Twist(Axis, Point, JointType)
%
%     |v|   | -Axis x Point  |
% xi =| | = |                |: for ROTATION joint.
%     |w|   |      Axis      |
%
%     |v|   |    Axis    |  
% xi =| | = |            |: for TRANSLATION joint.
%     |w|   |      0     |
%
% See also: .
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
%% 	xi = Joint2Twist(Axis, Point, JointType)
function xi = Joint2Twist(Axis, Point, JointType)
    if JointType == 'rot'
       xi = [-cross(Axis,Point); Axis];
    elseif JointType == 'tra'
       xi = [Axis; 0; 0; 0];
    else
       xi = [0; 0 ; 0: 0; 0; 0];
    end
%
    
    