%% ROTY2TFORM Convert to Homogeneous matrix b rotation around the Y axis 
%   H = ROTY2TFORM(B) converts an angle rotation B around Y axis into the
%   corresponding homogeneous matrix, H. B is a rotation angle in radians.
%
%   Example:
%       %Calculate the homogeneous matrix for a rotation angle b = pi/2
%       around Y axis.
%       Hyb = rotY2tform(b)
%       % Hyb = [cos(b) 0 sin(b) 0; 0 1 0 0; -sin(b) 0 cos(b) 0; 0 0 0 1] 
%       ans =
%           0.0000         0    1.0000         0
%                0    1.0000         0         0
%          -1.0000         0    0.0000         0
%                0         0         0    1.0000
%
% See also rotX2tform(b), rotZ2tform(g),
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
%% Hyb = rotY2tform(b)
function Hyb = rotY2tform(b)

    cb = cos(b);
    sb = sin(b);
    Hyb = [cb 0 sb 0; 0 1 0 0; -sb 0 cb 0; 0 0 0 1];
end

