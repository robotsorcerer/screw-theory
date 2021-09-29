%% "TWIST2TFORM" Convert a twist "xi" (6x1) to homogeneous matrix "E^" 3x4.
% Use in SE(3).
%
% 	tform = Twist2tform(xi)
%
% Returns the homogeneous "h" matrix 4x4 from a twixt "xi".
%    |v|         |W^ v|
% xi=| |  =>  E^=|    |  andn W^ = skew(W)
%    |W|         |    |
%
% See also: tform2twist, skew.
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
%% 	tform = Twist2tform(xi)
function tform = Twist2tform(xi)

    wr = skew([xi(4,1);xi(5,1);xi(6,1)]);
    tform = [wr(1,1) wr(1,2) wr(1,3) xi(1,1);
         wr(2,1) wr(2,2) wr(2,3) xi(2,1);
         wr(3,1) wr(3,2) wr(3,3) xi(3,1)];
end
%