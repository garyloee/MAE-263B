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

%% Initialization and defining coordinates

close all
clear variables
global currentQ
deg = pi/180;
% robot arm base at (0,0,0)
% Putting PCB center at (0,400,0)
PCB=[0,0,400,100];

% Reference configuration (0,400,50)
reference=[0,0,400,150];

% Feeder center at 
Feeder=[0,60,400,100];

% Chips at four corners of PCB
chip1=[0,-45,445,100];
chip2=[-90,45,445,100];
chip3=[-180,45,355,100];
chip4=[90,-45,355,100];

% via points
viaFeeder=[0,60,400,115];
via1=[0,-45,445,115];
via2=[-90,45,445,115];
via3=[-180,45,355,115];
via4=[-270,-45,355,115];

% initialize q for complete trajectories
q=[];
dq=[];
ddq=[];
%% Defining robot arm configuration
L(1) = Link([0,400,0,0,0]);
L(2) = Link([0,0,325,0,0]);
L(3) = Link([0,0,225,0,0]);
L(4) = Link([0, 0, 0,-pi,0]);
L(5) = Link([0, 0, 0,pi,1]);
L(6) = Link([0, -170, 0,0,0]);

Robot_263B = SerialLink(L, 'name', 'Robot');
points=[Feeder; chip1; chip2;chip3;chip4];

% Now we plot the robot with the angles given below

Robot_263B.model3d = 'UNIMATE/puma560';
Robot_263B

% base pose: 90 degrees elbow at the middle of the PCB 
% 230 mm off the ground

[t1,t2,t3,d4]=IKrobot(reference);% input z-axis rotation degree and px py pz
currentQ=[t1,t2,t3,d4];
Robot_263B.plot([0,currentQ,0],'workspace', [-400 400 -200 600 0 800])
for i=1:length(points)
    plotPoints(points(i,:));
end
pause(1)

% trajectory planning 
% input goal position with the first variable of the function
[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(Feeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(via1,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(chip1,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(via1,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(Feeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(via2,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(chip2,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(via2,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(Feeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(via3,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(chip3,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(via3,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(Feeder,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(viaFeeder,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(via4,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(chip4,q,dq,ddq);
[q,dq,ddq]=trajectoryGeneration(via4,q,dq,ddq);

[q,dq,ddq]=trajectoryGeneration(reference,q,dq,ddq);





% animation
for i=1:length(q)
    Robot_263B.plot([0,q(i,:),0],'workspace', [-1000 1000 -1000 1000 0 1000])

end
pause(1)

%totalTime=totalFrame*20;



plotting(q,dq,ddq);





clear L


%% functions
% input require coordinates
% IK function output joint angles





% master function
function [q,dq,ddq]=trajectoryGeneration(goal,q,dq,ddq)
global currentQ
[t1,t2,t3,d4]=IKrobot(goal);% input z-axis rotation degree and px py pz
[q_trj,dq_trj,ddq_trj]=RtrajectGen(t1, t2, t3, d4);
q=[q;q_trj];
dq=[dq;dq_trj];
ddq=[ddq;ddq_trj];
currentQ=[t1,t2,t3,d4];
%Robot_263B.plot([0,currentQ,0],'workspace', [-1000 1000 -1000 1000 0 1000])
end



% input transformation matrix and get the thetas and ds
function [t1, t2, t3, d4]=IKrobot(coordinates) % input z-axis degree and px py pz
t=num2cell(coordinates);
[rz, px, py, pz]=deal(t{:});
[TMatrix]=homoTransform(0, 0, rz, px, py, pz);
[t1, t2, t3, d4]=IK(TMatrix);
end

function [t1, t2, t3, d4]=IK(TMatrix)
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




function [TMatrix]=homoTransform(rx, ry, rz, px, py, pz)
zero = [0 0 0];

% T1

p = [px py pz]';
%2. ROTATION
RMatrix = rotz(rz)*roty(ry)*rotx(rx);
TMatrix = [RMatrix p;
        zero   1];
end


function [q_full,qd_full,qdd_full]=RtrajectGen(t1, t2, t3, d4)
global currentQ
q0 = currentQ;
qf = [t1 t2 t3 d4];

maxA=50*pi/180; % maximum acceleration
maxAcc=[maxA,maxA,10*maxA,150];

%S=max(abs(qf-q0)); % calculate maximum rotational joint displacement
S=abs(qf-q0);
t0=0;
tf=max(2*sqrt(S./maxAcc));

%tf=2*sqrt(S/maxA); % calculate total time from joint displacement
tf = round(10^1*tf)/10^1; % rounding up to simplify calculation
N=(tf-t0)*10; % 0.05 sencod a step
t=linspace(t0,tf,N);



%% Case1 Given qdd
des_qdd = maxAcc;
% tb = 0.5*(tf-t0) - 0.5*sqrt(qdd.^2*(tf-t0)^2 - 4*abs(qdd).*abs(qf-q0))./abs(2 *qdd);
%%% Case 2 Given tb
% tb = 0.5;
% qdd = abs((qf-q0)/(tb*(tf-t0-tb)));
q_full = [];
qd_full = [];
qdd_full = [];
% qd was set to be 0
for i = 1:4 % We have 6 joints
    
    qdd = des_qdd(i);
    tb = 0.5*(tf-t0) - 0.5*sqrt(qdd^2*(tf-t0)^2 - 4*abs(qdd)*abs(qf(i)-q0(i)))/abs(qdd);
    if q0(i) < qf(i)
        ab0 = [1 t0 t0^2;0 1 2 * t0;0 0 2] \ [q0(i) 0 qdd]'; % Coefficient for first blend
        abf = [1 tf tf^2;0 1 2 * tf;0 0 2] \ [qf(i) 0 -qdd]'; % Coefficient for second blend
    else
        ab0 = [1 t0 t0^2;0 1 2 * t0;0 0 2] \ [q0(i) 0 -qdd]'; % Coefficient for first blend
        abf = [1 tf tf^2;0 1 2 * tf;0 0 2] \ [qf(i) 0 qdd]'; % Coefficient for second blend
    end
    qb1 = ab0(1) + ab0(2) * tb + ab0(3) * tb^2;
    qb2 = abf(1) + abf(2) * (tf - tb) + abf(3) * (tf - tb)^2;
    a = [1 tb;1 (tf - tb)] \ [qb1;qb2]; % Coefficient for linear region
    % first parabolic region
    t11 = t((t0<=t) & (t<=t0+tb));
    q = ab0(1) + ab0(2)*t11 + ab0(3)*t11.^2;
    qd  = ab0(2) + 2*ab0(3)*t11;
    qdd = 2*ab0(3)*ones(size(t11));
    % Linear region
    t22 = t((t0+tb<t) & (t<tf-tb)); % linear region
    q = [q, a(1) + a(2)*t22];
    qd = [qd, a(2).*ones(size(t22))];
    qdd = [qdd, zeros(size(t22))];
    % second parabolic region
    t33 = t((tf-tb<=t) & (t<=tf)); 
    q   = [q, abf(1) + abf(2)*t33 + abf(3)*t33.^2];
    qd  = [qd, abf(2) + 2*abf(3)*t33];
    qdd = [qdd, 2*abf(3)*ones(size(t33))];
    q_full = [q_full q'];
    qd_full = [qd_full qd'];
    qdd_full = [qdd_full qdd'];
end

end


function plotting(q,dq,ddq)
totalFrame=linspace(1,length(q)*0.1,length(q));
figure(2); % Create figure 3

% Assuming totalFrame is your x-axis data and dq contains your y-axis data
yyaxis left; % Activates the left y-axis for the first three datasets
plot(totalFrame, q(:,1), 'b-', totalFrame, q(:,2), '-', totalFrame, q(:,3), 'c-'); % Plots the first three columns of dq on the left y-axis
ylabel('Roataional Joint Position [Rad]'); % Label for the left y-axis

yyaxis right; % Activates the right y-axis for the fourth dataset
plot(totalFrame, q(:,4)); % Plots the fourth column of dq (scaled down) on the right y-axis
ylabel('Prismatic Joint Position [mm]'); % Label for the right y-axis

% General plot formatting
title('Joint Position'); % Adds a title to your plot
xlabel('Time [s]'); % Label for the x-axis
legend({'q1', 'q2', 'q3', 'd4'}, 'Location', 'best'); % Adds a legend. Adjust the names as per your data.

% Optionally, set more properties
set(gca, 'FontSize', 24); % Sets font size of the axes labels and legend
grid on; % Turns on the grid


figure(3); % Create figure 3

% Assuming totalFrame is your x-axis data and dq contains your y-axis data
yyaxis left; % Activates the left y-axis for the first three datasets
plot(totalFrame, dq(:,1), 'b-', totalFrame, dq(:,2), '-', totalFrame, dq(:,3), 'c-'); % Plots the first three columns of dq on the left y-axis
ylabel('Rotaional Joint Velocity [Rad/s]'); % Label for the left y-axis

yyaxis right; % Activates the right y-axis for the fourth dataset
plot(totalFrame, dq(:,4)); % Plots the fourth column of dq (scaled down) on the right y-axis
ylabel('Prismatic Joint Velocity [mm/s]'); % Label for the right y-axis

% General plot formatting
title('Joint Velocity'); % Adds a title to your plot
xlabel('Time [s]'); % Label for the x-axis
legend({'q1', 'q2', 'q3', 'd4'}, 'Location', 'best'); % Adds a legend. Adjust the names as per your data.

% Optionally, set more properties
set(gca, 'FontSize', 24); % Sets font size of the axes labels and legend
grid on; % Turns on the grid


figure(4); % Create figure 3

% Assuming totalFrame is your x-axis data and dq contains your y-axis data
yyaxis left; % Activates the left y-axis for the first three datasets
plot(totalFrame, ddq(:,1), 'b-', totalFrame, ddq(:,2), '-', totalFrame, ddq(:,3), 'c-'); % Plots the first three columns of dq on the left y-axis
ylabel('Rotational Joint Acceleration [Rad/s^2]'); % Label for the left y-axis

yyaxis right; % Activates the right y-axis for the fourth dataset
plot(totalFrame, ddq(:,4)); % Plots the fourth column of dq (scaled down) on the right y-axis
ylabel('Prismatic Joint Acceleration [mm/s^2]'); % Label for the right y-axis

% General plot formatting
title('Joint Acceleration'); % Adds a title to your plot
xlabel('Time [s]'); % Label for the x-axis
legend({'q1', 'q2', 'q3', 'd4'}, 'Location', 'best'); % Adds a legend. Adjust the names as per your data.

% Optionally, set more properties
set(gca, 'FontSize', 24); % Sets font size of the axes labels and legend
grid on; % Turns on the grid
end


function plotPoints(point)
% Inputs: z-axis rotation angle (theta), and point coordinates (px, py, pz)
theta = point(1); % Example rotation angle in degrees
px = point(2); % Example x-coordinate of the point
py = point(3); % Example y-coordinate of the point
pz = point(4); % Example z-coordinate of the point

% Convert theta from degrees to radians for MATLAB trigonometric functions
theta_rad = theta*pi/180;
% Determine the length of the orientation line for visibility
lineLength = 15; % Adjust as needed for your scale

% Calculate the endpoint of the orientation line in the xy-plane
endX = px + lineLength * cos(theta_rad+pi/2);
endY = py + lineLength * sin(theta_rad+pi/2);

% Plot the point
hold on
plot3(px, py, pz, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % 'ko' for black circle marker
hold on; % Keep the plot active to add more elements

% Plot the orientation line
%plot3([px, endX], [py, endY], [pz, pz], 'r-', 'LineWidth', 100); % 'r-' for a red line
plot3(endX, endY, pz, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'b');

end









