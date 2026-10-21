%% DP2: State Feedback Control
%
% 41277 Control Design
% University of Technology Sydney, Australia
% Spring 2026
%
% A/Prof Ricardo P. Aguilera
%
% QUBE-Servo 2 rotary inverted pendulum, x = [theta alpha theta_dot alpha_dot]'
% (alpha = 0 upright). SFC design as in dp2_q2.m (Q2), observer added for Q3.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clearvars -except batch_case;   % batch_case = [linear_model run_observer], see run_all_cases.m

%% Continuous- and discrete-time model (Q1)
evalc('dp2_q1');    % Ac, Bc, Cc, Dc, A, B, C, D, ts = 1 ms and parameters
format short g

%% Simulation Settings
Tsim = 15;          % Simulation duration [s]
theta_ref = 5*pi/180;   % arm step amplitude, y1_ref Step block [rad]

% only run your linear model if you do not have a nonlinear one.
linear_model = 1;   % 1: control the linear model
                    % 0: control the nonlinear model

run_observer = 1;   % 0: run only SFC
                    % 1: run SFC + observer
if exist('batch_case', 'var')
    linear_model = batch_case(1);
    run_observer = batch_case(2);
end

% Plant starts at rest upright (integrator ICs = 0); the observer starts with
% a deliberate error (Remark 5) so its convergence is visible. With the observer,
% the step is delayed to t = 5 s so the start-up transient caused by x_hat(0)
% has decayed before the tracking test (Q2.5 keeps the step at t = 1 s).
x_hat_o = [3*pi/180 -1*pi/180 0 0]';
step_time = 1 + 4*run_observer;

save_figures = true;   % observer figures are exported to ../Report/assets/images

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
% Q2.3 design (dp2_q2.m): zeta = 0.6, wn = 2.5 rad/s dominant pair, two faster poles
ps = [-1.5+2i, -1.5-2i, -6, -8]

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

% Q3.2: about 5x faster than the fastest SFC pole (-8)
disp('Continuous-time observer eigenvalues:')
po_s = [-40, -45, -50, -55]
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
    mdl = 'sim_DP2_2026_SFC_Linear';
    load_system(mdl); set_param([mdl '/y1_ref'], 'Time', num2str(step_time));
    sim(mdl);
    close_system(mdl, 0);   % discard the step-time change, keep the saved model as is
    if run_observer
        plot_closed_loop(time, u(:,1), x_L, x_hat, theta_ref, step_time, 'Linear', save_figures);
    else
        run('plots_Linear_case.m');     % Q2.5 figures
    end
else
    disp("Simulating Non-Linear System case..." + newline)
    mdl = 'sim_DP2_2026_SFC_Nonlinear';
    load_system(mdl); set_param([mdl '/y1_ref'], 'Time', num2str(step_time));
    sim(mdl);
    close_system(mdl, 0);
    if run_observer
        plot_closed_loop(time, u(:,1), x_NL, x_hat, theta_ref, step_time, 'Nonlinear', save_figures);
    else
        run('plots_Nonlinear_case.m');  % Q2.5 figures
    end
end