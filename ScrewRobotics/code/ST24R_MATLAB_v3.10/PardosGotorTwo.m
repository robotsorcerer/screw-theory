%% "PARDOSGOTORTWO" Find the Inverse Kinematics of two consecutive
% TRANSLATION SCREWS (but both of the same type)
% which applied to a point move it ot a different point in space in SE(3).
%
% 	Theta1Theta2 = PardosTwo(x1, x2, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% In the case of TWO CONSECUTIVE TRANSLATIONS:
% the problen has ONE DOUBLE solution Theta1-Theta2 by Pardos-Gotor
% (even though is quite trivial) for the sake of resolving problems
% regardless of the joint type (rotation or translation)
% this computes the magnitude "t11-t21" (distance) of Screws, to move the
% point "p" to the point "k", but beware of the order for the movement,
% as the translations are applied on the twist x2 & subsequently on x1, so
% Point "p" is first moved to the point "c" by the Screw with Theta2
% and then moved from "c" to the point "k" by the Screw with Theta1.
%
% exp(E1^Theta1) * exp(E2^Theta2) * p = exp(E1^Theta1) * (c or d) = k
%          |v|
% Ei = xi =| | is 6x1 ; and p and k are points (3x1).
%          |w|
%             
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosOne, PardosThree, PardosFour
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
%% Theta1Theta2  = PardosGotorTwo(x1,x2,pp,pk)
function Theta1Theta2  = PardosGotorTwo(x1,x2,pp,pk)
%
    v1 = x1(1:3); 
    v2 = x2(1:3);
    pc = IntersectLines3D([v1 v2],[pk pp]);
    if norm(pc)==inf
        t11 = 0; t21 = 0;
    else
        t21 = v2'*(pc-pp);
        t11 = v1'*(pk-pc);
    end
    Theta1Theta2 = [t11 t21];
end 
%   
    
