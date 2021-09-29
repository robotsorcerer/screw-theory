%% BODYDH2TFORM Basic Body D-H (Denavit-Hartenberg) Matrix transformation
%   (Defined on mobile or tool system ? post-multiplication)
%   H = BODYDH2TFORM(DHPARAMS) gets the homogeneous matrix H for a single
%   rigid body (link) from its DH parameters dhparams = [d t r a] (1x4).
%       d: The translation on Z(i-1) axis (length units).
%       t: The rotation about Z(i-1) axis (radians). 
%       r: The length on Xi axis of the common normal (joint offset) in 
%          (length units).
%       a: The angle on Xi axis between successive Z(i-1) & Zi axes 
%          (joint twist) in (radians).
%   Only d and t are joint variables.
%
%   Example:
%       %Calculate the basic Matrix transformation for a body whose DH
%       parameters are dhparams = [d t r a]= [2 pi/2 3 pi/4] = .
%       Hdh = BodyDH2tform([2 pi/2 3 pi/4])
%       % Hdh1 = [ct -ca*st sa*st r*ct; st ca*ct -sa*ct r*st; 
%                  0 sa ca d; 0 0 0 1]
%       ans =
%           0.0000   -0.7071    0.7071    0.0000
%           1.0000    0.0000   -0.0000    3.0000
%                0    0.7071    0.7071    2.0000
%                0         0         0    1.0000
%
% See also trvZ2tform(p), rotZ2tform(g), trvX2tform(p), rotX2tform(a)
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
% You should have received a copy of the GNU Lesser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Hdh = BodyDH2tform(dhparams)
function Hdh = BodyDH2tform(dhparams)
    d = dhparams(1);
    t = dhparams(2);
    r = dhparams(3);
    a = dhparams(4);  
    Hdh = trvZ2tform(d)*rotZ2tform(t)*trvX2tform(r)*rotX2tform(a);
    % The direct solution is as follows
    %ct = cos(t);
    %st = sin(t);
    %ca = cos(a);
    %sa = sin(a); 
    %Hdh1 = [ct -ca*st sa*st r*ct; st ca*ct -sa*ct r*st; 0 sa ca d; 0 0 0 1];
end

