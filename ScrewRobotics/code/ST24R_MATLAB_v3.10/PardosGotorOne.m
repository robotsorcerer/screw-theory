%% "PARDOSGOTORONE" Find the Inverse Kinematics of a TRANSLATION SCREW
% when applied to a point in SE(3).
%
% Find the Inverse Kinematics of a single TRANSLATION SCREW
% when applied to a point in SE(3).
%
% 	Theta1 = PardosGotorOne(x1, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Compute the magnitude "Theta" (distance) of the Screw, to
% move the point "p" from its original position to the point "k" by
% TRANSLATION of the Twist x1.
%
%         |v|
% E = x1 =| | (6x1); and the points p and k are (3x1) coordinates. 
%         |w|
% exp(E^Theta) * p = k
%
% For a SCREW of pure TRANSLATION:
% the problen has one solution t1, which is proposed by Pardos-Gotor
% (even though is quite trivial) for the sake of resolving problems
% regardless of the joint type (rotation or translation)
% this computes the magnitude "Theta" (distance) of the Screw, to move the
% point "p" to the point "k" by the TRANSLATION of the Twist x1.
%
% IDEA for TRANSLATION:
% In fact it computes "Theta" to move a plane where is "p" to 
% another plane where is "k" and both planes are parallel, and “Theta” is
% the distance between both planes measured on the axis of x1.
% Beware that “p” and “k” can not coincide after the movement.
%
% IDEA for TRANSLATION:
% In fact it computes "Theta" to move a plane where is "p" to 
% another plane where is "k" and both planes are parallel, and “Theta” is
% the distance between both planes measured on the axis of x1.
% Beware that “p” and “k” can not coincide after the movement.
% but the algorithm works in the
% sense of giving the most approximate solution to the physical problem.
%
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosTwo, PardosThree, PardosFour
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
%% Theta1 = PardosGotorOne(x1, pp, pk)
%
function Theta1 = PardosGotorOne(x1, pp, pk)
%
    v1 = x1(1:3); w1 = x1(4:6);
    Theta1 = v1'*(pk-pp);
end
 %   
    
