%% A. Clear cache
clc;
clear all;
close;
%% B. Useful Constants
zero = [0 0 0];

%   sym(pi)     sym(-pi)    sym(pi/2)   sym(-pi/2)
%% ii. Forward Kinematics
syms t1 t2 t3 t4 t5 t6 real
syms a0 a1 a2 a3 L4 
syms a60j d1 d2 d3 d4 d5 df real

% DH parameters
alpha = [0 0 0 0 0]; % i-1 Create a symbolic representation of a constant
a     = [0 325 225 0 0]; % i-1
d     = [400 0 0 -d4 -170]; % i
theta = [t1 t2 t3 0 0]; % i

%DHpara=['alpha',alpha;'a',a;'d',d;'theta',theta]';
%disp(DHpara)

% Initialization of the transformation from the base to the end effector
T0_ee = eye(4);

for i = 1:length(alpha)
    fprintf('T (%d) to (%d) \n',i-1,i)
    T{i} = TF(a(i),alpha(i),d(i),theta(i)); % Cell
    pretty(simplify(T{i}))
    T0_ee = T0_ee*T{i}; % Sequential multiplications
end




T_res=(T{1}*T{2})^-1 % Equation Modification

%%
fprintf('T0_ee: \n')
T0_ee
fprintf('simplified T0_ee: \n')
simplify(T0_ee)
%
%% Functions
function T = TF(a,alpha,d,theta)
T = [cos(theta)            -sin(theta)            0           a
     sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
     sin(theta)*sin(alpha) cos(theta)*sin(alpha)  cos(alpha)  cos(alpha)*d
     0 0 0 1];
end