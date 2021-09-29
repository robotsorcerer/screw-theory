%% "PARDOSTWO" Find the Inverse Kinematics of two consecutive
% ROTATION or TRANSLATION SCREWS (but both of the same type)
% which applied to a point move it ot a different point in space in SE(3).
%
% 	Theta1Theta2 = PardosTwo(x1, x2, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
%
% Based on the work of Paden & Kahan subproblem two for INVERSE KINEMATICS.
% The problem could have up to TWO DOUBLE solutions for Theta1-Theta2:
% Theta1Theta2 = [t11 t21; t12 t22]
% as a consequence of the possible paths from p-c-k or p-d-k.
% The problem could have two solutions for each Theta, which is Theta-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% In the case of TWO CONSECUTIVE TRANSLATIONS:
% the problen has ONE DOUBLE solution Theta1-Theta2 by Pardos-Gotor
% (even though is quite trivial) for the sake of resolving problems
% regardless of the joint type (rotation or translation)
% this computes the magnitude "t11-t21" (distance) of Screws, to move the
% point "p" to the point "k", but beware of the order for the movement,
% as the translations are applied on the twist x2 & subsequently on x1, so
% Point "p" is first moved to the point "c" by the Screw with Theta2
% and then moved from "c" to the point "k" by the Screw with Theta1.
%
% See also: PadenKahanOne, PadenKahanThree
% See also: PardosOne, PardosTwo, PardosThree, PardosFour
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
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Theta1Theta2  = PadenKahanPardosTwo(x1,x2,pp,pk)
function Theta1Theta2  = PadenKahanPardosTwo(x1,x2,pp,pk)
%
    v1 = x1(1:3); w1 = x1(4:6);  
    v2 = x2(1:3); w2 = x2(4:6);
    if (norm(w1)&norm(w2)) == 0 % case of two consecutive TRANSLATIONS
        pc = IntersectLines3D([v1 v2],[pk pp]);
        if norm(pc)==inf
            t11 = 0; t21 = 0;            % NO SOLUTION
        else
            t21 = v2'*(pc-pp);
            t11 = v1'*(pk-pc);
        end
        Theta1Theta2 = [t11 t21; t11 t21];
    else                        % case of two consecutive ROTATIONS
        r1 = cross(w1,v1)/(norm(w1)^2);  
        r2 = cross(w2,v2)/(norm(w2)^2);
        pr = IntersectLines3D([w1 w2],[r1 r2]);
        if norm(pr)==inf
            t11 = 0; t21 = 0; t12 = 0; t22 = 0; % NO SOLUTION
        else
            % Calculate the a, b, g parameters for the PadenKahan2 solution
            u = pp - pr;
            v = pk - pr;
            Cw1w2 = cross(w1,w2);
            a = ((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
            b = ((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
            g2 = abs(real((norm(u)^2-a^2-b^2-2*a*b*w1'*w2)/norm(Cw1w2)^2));
            g = sqrt(g2);
            pc = pr + a*w1+b*w2+g*Cw1w2;
            pd = pr + a*w1+b*w2-g*Cw1w2;
            m = pc - pr;
            n = pd - pr;
            % Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne  
            up = u-w2*w2'*u;
            m2p = m-w2*w2'*m;
            m1p = m-w1*w1'*m;
            n2p = n-w2*w2'*n;
            n1p = n-w1*w1'*n;
            vp = v-w1*w1'*v;
            t21 = atan2(real(w2'*(cross(up,m2p))),real(up'*m2p));
            t11 = atan2(real(w1'*(cross(m1p,vp))),real(m1p'*vp));
            t22 = atan2(real(w2'*(cross(up,n2p))),real(up'*n2p));
            t12 = atan2(real(w1'*(cross(n1p,vp))),real(n1p'*vp));
            %
        end
        Theta1Theta2 = [t11 t21; t12 t22];
    end
end 
%   
    
