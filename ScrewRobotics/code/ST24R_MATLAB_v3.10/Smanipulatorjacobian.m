%% "SMANIPULATORJACOBIAN" computes the Spatial Manipulator Jacobian for a
% robot of any number of links.
% Use in SE(3).
%
% 	JstS = Smanipulatorjacobian(TwMag)
%
% SPATIAL MANIPULATOR JACOBIAN: At each configuration of theta, maps the
% joint velocity vector into the corresponding velocity of end effector.
% The contribution of the "ith" joint velocity to the end effector velicity
% is independent of the configuration of later joints in the chain.
% Thus, the "ith" column of jst is the "ith" joint twist, transformed to
% the current manipulator configuration.
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (link 1..n).
% Twn1..Twn6: The TWIST components for the joint SCREW movement.
% Magn: The MAGNITUDE component for the joint SCREW movement.
%
%               |v1 v2' ... vn'| 
% JstS(theta) = |              |   
%               |w1 w2' ... wn'|  
%           |vi'|
% With: Ei'=|   |=Ad                                       *Ei
%           |wi'|   (exp(E1^theta1)*...*exp(Ei-1^thetai-1))
%
% JstS(t)=Adg(t)*JstB(t) ; and Adg(t) is the Adjoint of gst(theta).
%
% See also: ForwardKinematicsPOE, expScrew, expAxAng
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
%% JstS = Smanipulatorjacobian(TwMag)
function JstS = Smanipulatorjacobian(TwMag)
    JstS = TwMag(1:6,:);
    PoE = expScrew(TwMag(:,1));
    for i = 2:size(TwMag,2)
        JstS(:,i) = Tform2adjoint(PoE)*TwMag(1:6,i);
        PoE = PoE*expScrew(TwMag(:,i)); 
    end
end