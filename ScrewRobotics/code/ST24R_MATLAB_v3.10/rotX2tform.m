%% ROTX2TFORM Convert to Homogeneous matrix a rotation around the X axis 
%   H = ROTX2TFORM(A) converts an angle rotation A around X axis into the
%   corresponding homogeneous matrix, H. A is a rotation angles in radians.
%
%   Example:
%       %Calculate the homogeneous matrix for a rotation angle a = pi/2
%       around X axis.
%       Hxa = rotX2tform(a)
%       % Hxa = [1 0 0 0; 0 cos(a) -sin(a) 0; 0 sin(a) cos(a) 0; 0 0 0 1] 
%       ans =
%       1.0000         0         0         0
%            0    0.0000   -1.0000         0
%            0    1.0000    0.0000         0
%            0         0         0    1.0000
%
% See also rotY2tform(b), rotZ2tform(g),
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
%% Hxa = rotX2tform(a)
function Hxa = rotX2tform(a)

    ca = cos(a);
    sa = sin(a);
    Hxa = [1 0 0 0; 0 ca -sa 0; 0 sa ca 0; 0 0 0 1];
end

