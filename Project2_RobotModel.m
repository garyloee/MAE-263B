%This script is based of the puma 560 manipulator code available in the Robotics Toolbox
%The script includes a set of modified, and a set of standard dh paramaters to create the robot


%MDL_PUMA560 Create model of Puma 560 manipulator
%
%      mdl_puma560
%
% Script creates the workspace variable p560 which describes the 
% kinematic and dynamic characteristics of a Unimation Puma 560 manipulator
% using standard DH conventions.
% The model includes armature inertia and gear ratios.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
% Reference::
% - "A search for consensus among model parameters reported for the PUMA 560 robot",
%   P. Corke and B. Armstrong-Helouvry, 
%   Proc. IEEE Int. Conf. Robotics and Automation, (San Diego), 
%   pp. 1608-1613, May 1994.
%
% See also SerialRevolute, mdl_puma560akb, mdl_stanford, mdl_twolink.

%
% Notes:
%    - the value of m1 is given as 0 here.  Armstrong found no value for it
% and it does not appear in the equation for tau1 after the substituion
% is made to inertia about link frame rather than COG frame.
% updated:
% 2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
%  fixed errors in COG for links 2 and 3
% 29/1/91 to agree with data from Armstrong etal.  Due to their use
%  of modified D&H params, some of the offsets Ai, Di are
%  offset, and for links 3-5 swap Y and Z axes.
% 14/2/91 to use Paul's value of link twist (alpha) to be consistant
%  with ARCL.  This is the -ve of Lee's values, which means the
%  zero angle position is a righty for Paul, and lefty for Lee.
%  Note that gravity load torque is the motor torque necessary
%  to keep the joint static, and is thus -ve of the gravity
%  caused torque.
%
% 8/95 fix bugs in COG data for Puma 560. This led to signficant errors in
%  inertia of joint 1. 
% $Log: not supported by cvs2svn $
% Revision 1.4  2008/04/27 11:36:54  cor134
% Add nominal (non singular) pose qn



% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

%% Initialization

close all
clear L
deg = pi/180;

%% Defining robot arm configuration
L(1) = Link([0,400,0,0,0]);
L(2) = Link([0,0,325,0,0]);
L(3) = Link([0,0,225,0,0]);
L(4) = Link([0, 0, 0,-pi,0]);
L(5) = Link([0, 0, 0,pi,1]);
L(6) = Link([0, -170, 0,0,0]);

Robot_263B = SerialLink(L, 'name', 'Robot');

% Now we plot the robot with the angles given below

Robot_263B.model3d = 'UNIMATE/puma560';

% base pose: 90 degrees elbow at the middle of the PCB
Robot_263B.plot([0,0,pi/2,-pi/2,0,0],'workspace', [-1000 1000 -1000 1000 0 1000])
Robot_263B

%% implementing IK
% input require coordinates
% IK function output joint angles


% input transformation matrix and get the thetas and ds
function [t1, t2, t3, d4]=IKrobot(TMatrix)
t = num2cell(TMatrix');
[   r11, r12, r13, px,...
    r21, r22, r23, py,...
    r31, r32, r33, pz,...
    ~,   ~,   ~,   ~,   ] = deal(t{:});
COSt2=(px^2+py^2-225^2-325^2)/(2*225*325);
t2=atan2(sqrt(1-COSt2^2),COSt2);
t1=atan2(py,px)-atan2(225*sin(t2),325+225*cos(t2));
t3=atan2(r21,r11)-t2-t1;
d4=230-pz;


end






















clear L
