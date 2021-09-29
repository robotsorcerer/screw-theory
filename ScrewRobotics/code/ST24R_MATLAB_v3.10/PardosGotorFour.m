%% "PARDOSGOTORFOUR" Find the Inverse Kinematics of two consecutive
% SCREWS whose axes are parallel (do not have a common point in space)
% which applied to a point move it ot a different point in space. In SE(3).
%
% 	Theta1Theta2 = PardosFour(x1, x2, pp, pk)
%
% In the case of TWO CONSECUTIVE ROTATIONS:
% Compute angles "Th2" and "Th1" of two subsequently applies SCREWS with
% corresponding twists x2 and x1, to move the point "p" to the point "k".
% Beware of the order for the movement, first applied x2 & subsequently x1.
% Point p is first moved to the point "c" or "d" by the Screw with Theta2
% and then moved from "c" or "d" to the point "k" by the Screw with Theta1.
%
% exp(E1^Theta1) * exp(E2^Theta2) * p = exp(E1^Theta1) * (c or d) = k
%          |v|
% Ei = xi =| | is 6x1 ; and p and k are points (3x1).
%          |w|
% Based on the work of Pardos subproblem for this type INVERSE KINEMATICS.
% The problem could have up to TWO DOUBLE solutions for Theta1-Theta2:
% Theta1Theta2 = [t11 t21; t12 t22]
% as a consequence of the possible paths from p-c-k or p-d-k.
% The problem could have two solutions for each Theta, which is Theta-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% See also: PadenKahanPardosOne, PadenKahanPardosTwo, PadenKahanPardosThree
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
%% Theta1Theta2  = PardosGotorFour(x1,x2,pp,pk)
%
function Theta1Theta2  = PardosGotorFour(x1,x2,pp,pk)
% First we get all the characteristics of the Screws on the plane
% perpendicular to the points "pp" and "pk".
    v1 = x1(1:3); w1 = x1(4:6);  
    v2 = x2(1:3); w2 = x2(4:6);
    r1 = cross(w1,v1)/(norm(w1)^2);  
    r2 = cross(w2,v2)/(norm(w2)^2);
    v = pk - r1; vw1 = w1*w1'*v; vp = v-vw1; nvp = norm(vp); c1 = r1+vw1;
    u = pp - r2; uw2 = w2*w2'*u; up = u-uw2; nup = norm(up); c2 = r2+uw2;
%
% Then we obtain the points "pc" and "pd" as the intersection of the two
% circumpherences defined by the radius "pp - c2" and "pk - c1".
    c2c1 = c2-c1; nc2c1 = norm(c2c1);
    wa = c2c1/nc2c1;
    wh = cross(w1,wa);
    if (nc2c1>=(nvp+nup)||nvp>=(nc2c1+nup)||nup>=(nc2c1+nvp))
        pc = c1 + nvp*wa;   % NO SOLUTION but gives aproximation.
        pd = pc;            % NO SOLUTION but gives aproximation.
    else
        a = (nc2c1^2-nup^2+nvp^2)/(2*nc2c1);
        h = sqrt(abs(nvp^2-a^2));
        pc = c1 + a*wa+h*wh;
        pd = c1 + a*wa-h*wh;
    end
%
% Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne approach.
    m1 = pc - r1; m1p = m1-w1*w1'*m1;
    n1 = pd - r1; n1p = n1-w1*w1'*n1;
    m2 = pc - r2; m2p = m2-w2*w2'*m2;
    n2 = pd - r2; n2p = n2-w2*w2'*n2;
    t11 = atan2(real(w1'*(cross(m1p,vp))),real(m1p'*vp));
    t12 = atan2(real(w1'*(cross(n1p,vp))),real(n1p'*vp));
    t21 = atan2(real(w2'*(cross(up,m2p))),real(up'*m2p));
    t22 = atan2(real(w2'*(cross(up,n2p))),real(up'*n2p));
%
    Theta1Theta2 = [t11 t21; t12 t22];
end 
%   
    
