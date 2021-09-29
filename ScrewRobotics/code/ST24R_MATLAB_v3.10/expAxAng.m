%% EXPAXANG Matrix Exponential of a Rigid Body ORIENTATION
% defined by the "Axis-Angle" (AxAng) – [x y z theta] == [W theta] (1x4)
% into a matrix "RotM = exp(W^theta)" (3x3)
% Use in SO(3).
%
% 	RotationMatrix = expAxAng(AxAng)
%
% Returns a rotation matrix from the rotation of magnitude "theta"
% around the "vector" W = [x y Z], using Rodrigues's formula: 
% rotm(W,Th)=exp(W^ * Th)=I + W^ * sin(Th)+ W^ * W^ * (1-cos(Th)) 
% Con W^=skew(W)
%
% See also: skew.
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
%% 	rotm = expAxAng(AxAng)
function rotm = expAxAng(AxAng)
    ws = skew(AxAng(1:3));
    t = AxAng(1,4);
rotm = eye(3)+ws*sin(t)+ws*ws*(1-cos(t));