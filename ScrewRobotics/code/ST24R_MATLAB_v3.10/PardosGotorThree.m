%% "PARDOSGOTORTHREE" Find the Inverse Kinematics of a TRANSLATION SCREW
% which applied to a point move it to a
% certain distance from another point in space in SE(3).
%
% 	Theta1 = PardosThree(x1, pp, pk, de)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Compute the magnitude Theta1 of the Screw with twist x1, to move the
% point "pp" from its position in 3D to another point "c" or "e" complying
% that the distance between "c" or "e" to the point "pk" is "de". 
% 
%         |v|
% E = x1 =| | (6x1); the points pp, pk, c and e are (3x1). 
%         |w|
% exp(E^Theta) * pp = c; and de = norm(c-pk) and de = norm(e-pk).
%
% For a SCREW of pure TRANSLATION:
% Based on a solution proposed by Pardos-Gotor subproblem Three for IK.
% The problem could has zero, one or two solutions, Theta1 = [t11; t12]
% (even though is quite trivial) for the sake of solving problems
% regardless of the joint type (rotation or translation)
% this computes the magnitude "Theta" (distance) of the Screw, to move the
% point "p" to the point "c" or "e" by the TRANSLATION of the Twist x1. 
% In fact computes "Theta" to move a plane where is "p" to another planes 
% where is "c" or "e" and all planes are perpendicular to the axis of x1.
%
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosOne, PardosTwo, PardosFour
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
%% Theta1 = PardosThree(x1, pp, pk, de)
function Theta1 = PardosGotorThree(x1, pp, pk, de)
%
   v1 = x1(1:3);
   % Subproblem by the intersection or line pp-v1 with sphere pk-de.
   kmp = pk - pp;
   kmpp = v1'*kmp;
   t11 = kmpp; t12 = kmpp; % NO SOLUTION
   root = real(kmpp^2-norm(kmp)^2+de^2);
   if root > 0             % THERE IS SOLUTION
       t11 = t11+sqrt(root);
       t12 = t12-sqrt(root);
   end
   Theta1 = [t11; t12];
end
%