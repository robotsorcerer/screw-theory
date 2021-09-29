%% TRVX2TFORM Convert to Homogeneous matrix a translation on X axis 
%   H = TRVX2TFORM(P) converts a translation P on X axis into the
%   corresponding homogeneous matrix H. P is a position in longitude units.
%
%   Example:
%       %Calculate the homogeneous matrix for a translation p = 2
%       on X axis.
%       Hxp = trvX2tform(p)
%       % Hxp = [1 0 0 p; 0 1 0 0; 0 0 1 0; 0 0 0 1] 
%       ans =
%                1         0         0         2
%                0         1         0         0
%                0         0         1         0
%                0         0         0         1
%
% See also trvY2tform(p), trvZ2tform(p),
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
%% Hxp = trvX2tform(pg)
function Hxp = trvX2tform(p)

    Hxp = [1 0 0 p; 0 1 0 0; 0 0 1 0; 0 0 0 1];
end

