%% ROTZ2TFORM Convert to Homogeneous matrix a rotation around the Z axis 
%   H = ROTZ2TFORM(G) converts an angle rotation G around Z axis into the
%   corresponding homogeneous matrix, H. G is a rotation angle in radians.
%
%   Example:
%       %Calculate the homogeneous matrix for a rotation angle g = pi/2
%       around Z axis.
%       Hzg = rotZ2tform(g)
%       % Hzg = [cos(g) -sin(a) 0 0; sin(a) cos(a) 0 0; 0 0 1 0; 0 0 0 1] 
%       ans =
%       0.0000   -1.0000         0         0
%       1.0000    0.0000         0         0
%            0         0    1.0000         0
%            0         0         0    1.0000
%
% See also rotX2tform(a), rotY2tform(b),
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
%% Hzg = rotZ2tform(g)
function Hzg = rotZ2tform(g)

    cg = cos(g);
    sg = sin(g);
    Hzg = [cg -sg 0 0; sg cg 0 0; 0 0 1 0; 0 0 0 1];
end

