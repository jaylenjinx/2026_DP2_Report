%% One-off setup of the provided DP2 Simulink models for the QUBE-Servo 2 RIP
% - Digital Controller block: SFC + Luenberger observer, single input, +/-10 V
% - References: y1_ref steps to theta_ref at t = 1 s, y2_ref = alpha_ref = 0
% - Nonlinear Model: replaced by rip_nonlinear_dynamics + integrator
% Run once (models are saved); main_SFC.m then just simulates them.

controller_code = strjoin({
'function [u_k, x_hat] = SFC(y_ref,y_k, x_k,F,run_observer, Lo, A, B, C, x_hat_o)'
''
'persistent x_hat_k'
'if isempty(x_hat_k)'
'    x_hat_k = x_hat_o;'
'end'
'x_hat = x_hat_k;'
''
'%% System size'
'n = length(A);      % number of states'
'm = size(B)*[0 1]''; % number of inputs'
'p = size(C)*[1 0]''; % number of outputs'
''
'%% State References (Q2.2)'
'% Steady state of x(k+1) = A x(k) + B u(k), y = C x: [A-I B; C 0][x*; u*] = [0; y*]'
'xu_ref = [A-eye(n) B; C zeros(p,m)] \ [zeros(n,1); y_ref];'
'x_ref = xu_ref(1:n);'
'u_ref = xu_ref(n+1:n+m);'
''
'u_k = zeros(m,1); %clear input vector before running SFC (DO NOT DELETE)'
'if (run_observer == 0)'
'    %% SFC with full state measurement (Q2.5)'
'    u_k = -F*(x_k - x_ref) + u_ref;'
'else'
'    %% SFC with estimated state (Q3.4)'
'    u_k = -F*(x_hat_k - x_ref) + u_ref;'
'end'
''
'%% Input saturation: QUBE-Servo 2 recommended amplifier range +/-10 V'
'if (u_k(1) > 10)'
'    u_k(1) = 10;'
'end'
'if (u_k(1) < -10)'
'    u_k(1) = -10;'
'end'
''
'%% Luenberger observer (Q3.3): x_hat(k+1) = A x_hat(k) + B u(k) + L (y(k) - y_hat(k))'
'y_hat = C*x_hat_k;'
'x_hat_k = A*x_hat_k + B*u_k + Lo*(y_k - y_hat);'
''
'end'
}, newline);

models = {'sim_DP2_2026_SFC_Linear', 'sim_DP2_2026_SFC_Nonlinear'};
for i = 1:numel(models)
    mdl = models{i};
    load_system(mdl);

    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/Digital Controller' newline 'Sampling at Ts']);
    ch.Script = controller_code;

    set_param([mdl '/y1_ref'], 'Time', '1', 'Before', '0', 'After', 'theta_ref');
    set_param([mdl '/y2_ref'], 'Value', '0');

    if contains(mdl, 'Nonlinear')
        build_nonlinear_subsystem([mdl '/Nonlinear Model']);
    end

    save_system(mdl);
    close_system(mdl);
end
disp('Simulink models updated.')

function build_nonlinear_subsystem(sub)
% Replace the template's placeholder dynamics with the RIP model:
%   x_dot = f(x, u)  ->  integrator (x(0) = x0_NL)  ->  y = Cc x
lines = find_system(sub, 'LookUnderMasks', 'all', 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
delete_line(lines);
blocks = find_system(sub, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(blocks)
    if ~strcmp(blocks{k}, sub) && ~any(strcmp(get_param(blocks{k}, 'BlockType'), {'Inport', 'Outport'}))
        delete_block(blocks{k});
    end
end

add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/RIP dynamics'], 'Position', [120 40 260 110]);
add_block('simulink/Continuous/Integrator', [sub '/Integrator'], 'InitialCondition', 'x0_NL', 'Position', [320 55 350 95]);
add_block('simulink/Math Operations/Gain', [sub '/Cc'], 'Gain', 'Cc', 'Multiplication', 'Matrix(K*u)', 'Position', [420 25 470 65]);

ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [sub '/RIP dynamics']);
ch.Script = strjoin({
    'function x_dot = f(x, u, P_rip)'
    'x_dot = rip_nonlinear_dynamics(x, u, P_rip);'
    }, newline);
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'P_rip');
d.Scope = 'Parameter';

set_param([sub '/u'], 'Position', [20 83 50 97]);
set_param([sub '/y_NL'], 'Position', [530 38 560 52]);
set_param([sub '/x_NL'], 'Position', [530 98 560 112]);

add_line(sub, 'u/1', 'RIP dynamics/2', 'autorouting', 'on');
add_line(sub, 'RIP dynamics/1', 'Integrator/1', 'autorouting', 'on');
add_line(sub, 'Integrator/1', 'RIP dynamics/1', 'autorouting', 'on');
add_line(sub, 'Integrator/1', 'Cc/1', 'autorouting', 'on');
add_line(sub, 'Cc/1', 'y_NL/1', 'autorouting', 'on');
add_line(sub, 'Integrator/1', 'x_NL/1', 'autorouting', 'on');
end
