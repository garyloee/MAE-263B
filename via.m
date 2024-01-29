clc;
clear all;
close;

c1=[5,95,0];
c2=[95,95,0];
c3=[5,5,0];
c4=[95,5,0];
c=[50,50,0];
f=[110,50,0];

v1=(c1+f)/2;
v2=(c2+f)/2;
v3=(c3+f)/2;
v4=(c4+f)/2;
v5=(c+f)/2;



zero = [0 0 0];

% T1
alp = 0; %z
bet = 0;  %y
gam = 0;  %x
p_01 = v1';
%2. ROTATION
R_01 = rotz(alp)*roty(bet)*rotx(gam);
T_01 = [R_01 p_01;
        zero   1];

% T2
alp = -45; %z
bet = 0;  %y
gam = 0;  %x
p_02 = v2';
%2. ROTATION
R_02 = rotz(alp)*roty(bet)*rotx(gam);
T_02 = [R_02 p_02;
        zero   1];
% T3
alp = -90; %z
bet = 0;  %y
gam = 0;  %x
p_03 = v3';
%2. ROTATION
R_03 = rotz(alp)*roty(bet)*rotx(gam);
T_03 = [R_03 p_03;
        zero   1];

% T3
alp = -135; %z
bet = 0;  %y
gam = 0;  %x
p_04 = v4';
%2. ROTATION
R_04 = rotz(alp)*roty(bet)*rotx(gam);
T_04 = [R_04 p_04;
        zero   1];

% T3
alp = 0; %z
bet = 0;  %y
gam = 0;  %x
p_05 = v5';
%2. ROTATION
R_05 = rotz(alp)*roty(bet)*rotx(gam);
T_05 = [R_05 p_05;
        zero   1];

% transformation result
T_01
T_02
T_03
T_04
T_05

%T_transform = T_01 * T_02 * T_03 % CAREFUL WITH EQUATION

%fprintf('***double check equation***\n')