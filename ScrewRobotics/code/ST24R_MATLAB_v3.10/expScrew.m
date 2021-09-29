%% EXPSCREW Matrix Exponential of a  Rigid Body Motion by SCREW movement
% defined by the "Twist-Mangitude" (TwMag) – [xi; theta] (7x1)
% where the TWIST "xi" (6x1) and its MAGNITUDE "theta"
% into a matrix "g = exp(xi^theta)" (4x4).
% Use in SE(3).
%
%   g = expScrew(TwMag)
%
% Returns a homogeneous "g" matrix (4x4)
%    |v| 
% xi=| |   
%    |W|  
%              |exp(W^Th)  (I-exp(W^Th))*(W x v)+W*W'*v*Theta|
% exp(E^Theta)=|                                             |:if W not 0.
%              |0                          1                 |
%              |I v*Theta|
% exp(E^Theta)=|         |: if W is Zero.
%              |0     1  |
% exp(E^Theta)=gst(theta)*gst(0)^-1                                            
% Use Rodrigues's formula: 
% exp(W^Th)=I + W^ * sin(Th)+ W^ * W^ * (1-cos(Th)) 
% With W^=skew(W), which means W x v == cross product W^ * v.
%
% BE AWARE that a value theta = 0 produces a result g=I.
%
% See also: expAxAng.
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
%% 	g = expScrew(TwMag)
function g = expScrew(TwMag)
    v = TwMag(1:3); % "vee" component of the TWIST.
    w = TwMag(4:6); % "omega" component of the TWIST.
    t = TwMag(7); % "theta" magnitude component of the SCREW.
    if norm(w) == 0 % only translation
       r = eye(3);
       p = v*t;
    else
       r = expAxAng([w' t]);
       p = (eye(3)-r)*(cross(w,v))+w*w'*v*t;
    end
g = [r, p; [0 0 0 1]];
    
    