%% DP2: State Feedback Control
%
% 41277 Control Design
% University of Technology Sydney, Australia
% Spring 2026
%
% A/Prof Ricardo P. Aguilera
%
% QUBE-Servo 2 rotary inverted pendulum, x = [theta alpha theta_dot alpha_dot]'
% (alpha = 0 upright). Run setup_simulink_models.m once before this file.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clearvars -except batch_case;   % batch_case = [linear_model run_observer], see run_all_cases.m

%% Continuous- and discrete-time model (Q1)
evalc('dp2_q1');    % Ac, Bc, Cc, Dc, A, B, C, D, ts = 1 ms and parameters
format short g

%% Simulation Settings
Tsim = 10;          % Simulation duration [s]
theta_ref = deg2rad(30);    % rotary arm step reference at t = 1 s [rad]

% only run your linear model if you do not have a nonlinear one.
linear_model = 1;   % 1: control the linear model
                    % 0: control the nonlinear model

run_observer = 1;   % 0: run only SFC
                    % 1: run SFC + observer
if exist('batch_case', 'var')
    linear_model = batch_case(1);
    run_observer = batch_case(2);
end

% Plant starts at rest upright; the observer starts with a deliberate error
% (Remark 5) so its convergence is visible.
x0_NL   = [0 0 0 0]';
x_hat_o = [deg2rad(10) deg2rad(-5) 0 0]';

save_figures = true;   % export PNGs to ../Report/assets/images

%% Continuous-time state space model
disp('*************************')
disp('Continuous-Time Matrices:')
disp('x_dot(t)=Ac*x(t)+Bc*u(t)')
disp('    y(t)=Cc*x(t)+Dc*u(t)')
n = size(Ac,1)      % number of states
m = size(Bc,2)      % number of inputs
p = size(Cc,1)      % number of outputs
Ac, Bc, Cc, Dc

%Continuous-time model in a compact form
sys_ct=ss(Ac,Bc,Cc,Dc);
disp('Open-loop eigenvalues:')
eig(Ac)

disp('*************************')
disp('Controllability Matrix')
CM=ctrb(Ac,Bc)

if (rank(CM)==n)
    disp('System is controllable!!!')
end

%% Discrete-Time Model
%    x(k+1) = Ax(k)+Bu(k)
%      y(k) = Cx(k)+Du(k)
disp('***********************')
disp('Discrete-Time Matrices:')
disp('x(k+1)=A*x(k)+B*u(k)')
disp('  y(k)=C*x(k)')
A, B, C, D

%% SFC
disp('****************************************')
disp('Continuous-time closed-loop eigenvalues:')
Ts_req = 2;     % design settling time [s] (requirement < 5 s)
OS_req = 5;     % design overshoot [%]   (requirement < 15 %)
zeta = -log(OS_req/100)/sqrt(pi^2 + log(OS_req/100)^2)
wn   = 4/(zeta*Ts_req)
ps = [-zeta*wn + 1j*wn*sqrt(1-zeta^2), -zeta*wn - 1j*wn*sqrt(1-zeta^2), -30, -35]

disp('Discrete-time closed-loop eigenvalues:')
pz = exp(ps*ts)

disp('Discrete-time feedback gain:')
F=place(A,B,pz)

disp('Closed-loop eigenvalues:')
eig(A-B*F)

%% Observer
disp('****************************************')
disp('Observability Matrix')
OM=obsv(Ac,Cc)

if (rank(OM)==n)
    disp('System is observable!!!')
end

disp('Continuous-time observer eigenvalues:')
po_s = [-100, -110, -120, -130]
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
    P_rip = [a b c h mp*lp^2 Dr Dp kt km Rm];
    sim('sim_DP2_2026_SFC_Nonlinear.slx');
    % modify the 'plots_Nonlinear_case.m' file to generate your plots
    run('plots_Nonlinear_case.m');
end
