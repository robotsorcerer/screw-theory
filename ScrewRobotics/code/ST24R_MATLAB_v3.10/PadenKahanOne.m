%% "PADENKAHANONE" Find the Inverse Kinematics of a single ROTATION SCREW
% when applied to a point in SE(3).
%
% 	Theta1 = PadenKahanOne(x1, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Compute the magnitude "Theta" (angle) of the Screw, to
% move the point "p" from its original position to the point "k" by
% the ROTATION of the Twist x1.
%
%         |v|
% E = x1 =| | (6x1); and the points p and k are (3x1) coordinates. 
%         |w|
% exp(E^Theta) * p = k
%
% For a SCREW of pure ROTATION:
% Based on the work of Paden & Kahan subproblem ONE for INVERSE KINEMATICS.
% For a SCREW of pure ROTATION the problem has two solutions t1 and t1-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics and calculated as follows:
% switch sign(t11)
%    case -1
%        t12 = 2*pi+t11;
%    case 0
%        t12 = 2*pi;
%    case 1
%        t12 = t11-2*pi;
%    otherwise
%        t12 = NaN;
%    end
%
% IDEA for ROTATION:
% In fact it computes "Theta" to rotate the vector u' around x1
% to make it parallel to the vector v', but "p" and "k" can be in
% different planes and even u' and v' be of different module, but o course
% both planes are parallel & perpendicular to the axis of x1, then “p” and
% “k” can not coincide after the movement, but the algorithm works in the
% sense of giving the most approximate solution to the physical problem.
%
% See also: PadenKahanTwo, PadenKahanThree
% See also: PardosOne, PardosTwo, PardosThree, PardosFour
% See also: PadenKahanPardosOne, PadenKahanPardosTwo, PadenKahanPardosThree
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
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Theta1 = PadenKahanOne(x1, pp, pk)
function Theta1 = PadenKahanOne(x1, pp, pk)
%
    v1 = x1(1:3); w1 = x1(4:6);
    r1 = cross(w1,v1)/(norm(w1)^2);
    u = pp-r1; up = u-w1*w1'*u;
    v = pk-r1; vp = v-w1*w1'*v;
    Theta1 = atan2(w1'*(cross(up,vp)),up'*vp);
end
 %   
    
