%% "skew" Generate a skew symmetric matrix from an axis.
% Use in SO(3).
%
% 	sk = skew(w)
%
% Returns a skew symmetric matrix s 3x3 from the vector 3x1 w[a1;a2;a3;].
%     |0  -a3  a2| 
% sk =|a3   0 -a1|
%     |-a2 a1   0|
% See also: unskew.
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
%% 	sk = skew(W)
function sk = skew(w)
sk = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];