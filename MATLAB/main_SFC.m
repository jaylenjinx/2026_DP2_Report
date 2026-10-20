%% DP2: State Feedback Control
% 
% 41277 Control Design
% University of Technology Sydney, Australia
% Spring 2026
%
% A/Prof Ricardo P. Aguilera
%
% Enjoy it!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear;

%% Simulation Settings
ts = 1/100;         % select a proper sampling time for your setup
Tsim = 10;          % Simulation duration

% only run your linear model if you do not have a nonlinear one.
linear_model = 1;   % 1: control the linear model
                    % 0: control the nonlinear model

run_observer = 0;   % 0: run only SFC
                    % 1: run SFC + observer
                    

% Observer initialization. 
% Change this vector size according to the number of states in your setup
x_hat_o = [0.1 -0.1]';

%% Continuous-time state space model
disp('*************************')
disp('Continuous-Time Matrices:')
disp('x_dot(t)=Ac*x(t)+Bc*u(t)')
disp('    y(t)=Cc*x(t)+Dc*u(t)')
n = 2; %number of states

Ac=[ 0    -1;
    1   -1]

Bc=[1 0
    0 1]'

Cc=[1 0
    0 1];

Dc=0

%Continuous-time model in a compact form
sys_ct=ss(Ac,Bc,Cc,Dc); 

disp('*************************')
disp('Controllability Matrix')
CM=ctrb(Ac,Bc)

if (rank(CM)==n)
    disp('System is controllable!!!')
end

%% Discrete-Time Model
%    x(k+1) = Ax(k)+Bu(k)
%      y(k) = Cx(k)+Du(k)                 
                    
sys_dt = c2d(sys_ct,ts);
disp('***********************')
disp('Discrete-Time Matrices:')
disp('x(k+1)=A*x(k)+B*u(k)')
disp('  y(k)=C*x(k)')
A=sys_dt.A
B=sys_dt.B
C=sys_dt.C
D=sys_dt.D                 


%% SFC
disp('****************************************')
disp('Continuous-time closed-loop eigenvalues:')
Ts = 1; % settling time
OS = 5; % overshoot percentage
ps = [-1+j, -1-j]

disp('Discrete-time closed-loop eigenvalues:')
pz = exp(ps*ts)

disp('Discrete-time feedback gain:')
F=place(A,B,pz)

disp('Closed-loop eigenvalues:')
eig(A-B*F)

%% Observer
disp('****************************************')
disp('Continuous-time observer eigenvalues:')
po_s = [-10+j, -10-j]
disp('Discrete-time observer eigenvalues:')
po_z = exp(po_s*ts)

disp('Discrete-time observer gain:')
Lo=place(A',C',po_z)'

disp('Observer eigenvalues:')
eig(A-Lo*C)

%% Simulation
disp(newline + "*******************************")
warning('off','all')
if (linear_model==1)
    disp("Simulating Linear System case..." + newline)
    sim('sim_DP2_2026_SFC_Linear.slx');
    % modify the 'plots_Linear_case.m' file to generate your plots
    run('plots_Linear_case.m');
else
    disp("Simulating Non-Linear System case..." + newline)
    sim('sim_DP2_2026_SFC_Nonlinear.slx');
    % modify the 'plots_Nonlinear_case.m' file to generate your plots
    run('plots_Nonlinear_case.m');
end