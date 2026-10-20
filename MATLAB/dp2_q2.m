%% DP2 Q2: State-Feedback Control
% State order: [theta; alpha; theta_dot; alpha_dot]
% Input: motor voltage (V)
% Upright pendulum: alpha = 0

%clear;
clc;
close all;

%% Load the Q1 model
% dp2_q1.m defines Ac, Bc, Cc, Dc, A, B, C, D and ts.
run('dp2_q1.m');

n = size(A,1);

assert(isequal(size(A),[4 4]), 'A must be 4-by-4.');
assert(isequal(size(B),[4 1]), 'B must be 4-by-1.');
assert(isequal(size(C),[2 4]), 'C must be 2-by-4.');
assert(abs(ts - 0.001) < 1e-12, ...
    'This design assumes a sampling period of 0.001 s.');

%% Q2.1: Controllability
CMc = ctrb(Ac,Bc);
CMd = ctrb(A,B);

disp('Continuous-time controllability matrix:');
disp(CMc);

fprintf('Continuous-time controllability rank: %d\n',rank(CMc));
fprintf('Discrete-time controllability rank:   %d\n',rank(CMd));

assert(rank(CMc) == n, ...
    'The continuous-time system is not completely controllable.');

assert(rank(CMd) == n, ...
    'The discrete-time system is not completely controllable.');

%% Q2.2: State and input references
% Illustrative command: 5-degree arm-angle step.
% Change this if the assignment specifies another amplitude.

theta_command = 5*pi/180;

y_star = [theta_command; 0];
x_star = [theta_command; 0; 0; 0];
u_star = 0;

disp('State reference x_star:');
disp(x_star);

disp('Input reference u_star:');
disp(u_star);

equilibrium_residual = (A-eye(n))*x_star + B*u_star;
output_residual = C*x_star + D*u_star - y_star;

assert(norm(equilibrium_residual) < 1e-10, ...
    'The selected reference is not an equilibrium.');

assert(norm(output_residual) < 1e-10, ...
    'The equilibrium does not produce the desired output.');

%% Q2.3: Select closed-loop poles
% Requirements reproduced in the report:
% Overshoot < 15%
% Settling time < 5 s
%
% Use a 2% settling band.
% Dominant second-order design: zeta = 0.6, wn = 2.5 rad/s.
% These estimates must be checked against the full response.

OS_limit = 15;
settling_limit = 5;

zeta = 0.6;
wn = 2.5;

OS_estimate = 100*exp(-pi*zeta/sqrt(1-zeta^2));
settling_estimate = 4/(zeta*wn);

ps = [-1.5+2i, -1.5-2i, -6, -8];
pz = exp(ps*ts);

fprintf('\nSecond-order overshoot estimate: %.3f %%\n',OS_estimate);
fprintf('Second-order settling estimate:  %.3f s\n',settling_estimate);

disp('Selected continuous-time poles:');
disp(ps.');

disp('Selected discrete-time poles:');
disp(pz.');

%% Q2.4: Calculate state-feedback gain
% Control law:
% u[k] = -F*(x[k] - x_ref[k]) + u_star

F = place(A,B,pz);

closed_loop_poles = eig(A-B*F);

disp('State-feedback gain F:');
disp(F);

disp('Achieved closed-loop poles:');
disp(closed_loop_poles);

assert(all(abs(closed_loop_poles) < 1), ...
    'The nominal discrete closed loop is unstable.');

pole_error = max(arrayfun( ...
    @(z) min(abs(closed_loop_poles-z)), pz));

assert(pole_error < 1e-8, ...
    'Achieved poles do not match the requested poles.');

%% Q2.5: Nominal discrete-model simulation
% This loop tests the discrete state-space model directly.
% It does not run the uploaded Simulink model.

Tsim = 15;
step_time = 1;

time = (0:ts:Tsim).';
N = numel(time);

theta_ref = theta_command*double(time >= step_time);

x = zeros(N,n);
x(1,:) = [0 0 0 0];

u = zeros(N,1);
u_requested = zeros(N,1);

voltage_limit = 10;

for k = 1:N

    x_ref = [theta_ref(k); 0; 0; 0];

    u_requested(k) = -F*(x(k,:).' - x_ref) + u_star;

    % One motor input with bipolar voltage limits.
    u(k) = min(voltage_limit, ...
               max(-voltage_limit,u_requested(k)));

    if k < N
        x(k+1,:) = (A*x(k,:).' + B*u(k)).';
    end
end

y = (C*x.' + D*u.').';

theta = y(:,1);
alpha = y(:,2);

assert(all(isfinite(x(:))) && all(isfinite(u)), ...
    'The response contains non-finite values.');

%% Calculate performance
% Settling time is measured from the command step.
% Overshoot is relative to the commanded arm-angle change.

post_step = find(time >= step_time);

theta_after_step = theta(post_step);
normalised_response = theta_after_step/theta_command;

overshoot_percent = max(0, ...
    100*(max(normalised_response)-1));

settling_band = 0.02*abs(theta_command);

outside_band = find( ...
    abs(theta_after_step-theta_command) > settling_band);

if isempty(outside_band)

    settling_time = 0;

elseif outside_band(end) == numel(post_step)

    settling_time = NaN;

else

    first_settled_index = ...
        post_step(outside_band(end)+1);

    settling_time = time(first_settled_index)-step_time;

end

final_error_rad = theta_command-theta(end);
peak_alpha_deg = max(abs(alpha))*180/pi;
peak_voltage = max(abs(u));

% Each command except the final sample is applied for one interval.
saturation_time = ts*sum( ...
    abs(u_requested(1:end-1)) > voltage_limit);

fprintf('\nQ2 nominal discrete-model results\n');
fprintf('Arm command:          %.3f deg\n',theta_command*180/pi);
fprintf('Overshoot:            %.3f %%\n',overshoot_percent);
fprintf('2%% settling time:     %.3f s\n',settling_time);
fprintf('Final arm error:      %.6g rad\n',final_error_rad);
fprintf('Peak pendulum angle:  %.4f deg\n',peak_alpha_deg);
fprintf('Peak motor voltage:   %.4f V\n',peak_voltage);
fprintf('Time in saturation:   %.4f s\n',saturation_time);

if overshoot_percent < OS_limit && ...
        isfinite(settling_time) && ...
        settling_time < settling_limit

    disp('PASS: Both stated tracking requirements are met.');

else

    disp('Both tracking requirements have not been demonstrated.');

end

%% Plot outputs and control input
figure('Color','w','Name','Q2 closed-loop response');

subplot(3,1,1);
plot(time,theta*180/pi,'b','LineWidth',1.3);
hold on;
plot(time,theta_ref*180/pi,'r--','LineWidth',1.2);
ylabel('\theta (deg)');
title('Nominal discrete-model state-feedback response');
legend('Arm angle','Reference','Location','best');
grid on;

subplot(3,1,2);
plot(time,alpha*180/pi,'b','LineWidth',1.3);
ylabel('\alpha (deg)');
title('Pendulum angle from upright');
grid on;

subplot(3,1,3);
plot(time,u,'b','LineWidth',1.3);
ylabel('Voltage (V)');
xlabel('Time (s)');
grid on;

%% Plot arm-angle transient and settling band
figure('Color','w','Name','Q2 transient zoom');

plot(time,theta*180/pi,'b','LineWidth',1.3);
hold on;

plot(time,theta_ref*180/pi,'r--','LineWidth',1.2);

yline(1.02*theta_command*180/pi,'k:', ...
    'Upper 2% limit');

yline(0.98*theta_command*180/pi,'k:', ...
    'Lower 2% limit');

if isfinite(settling_time)
    xline(step_time+settling_time,'g--', ...
        'Settling instant');
end

xlim([step_time step_time+5]);
xlabel('Time (s)');
ylabel('\theta (deg)');
title('Arm-angle transient');
grid on;

%% Plot voltage limits
figure('Color','w','Name','Q2 voltage limits');

plot(time,u,'b','LineWidth',1.3);
hold on;

yline(voltage_limit,'r--','+10 V');
yline(-voltage_limit,'r--','-10 V');

xlabel('Time (s)');
ylabel('Motor voltage (V)');
title('Applied control voltage');
grid on;

%% Save results
save('dp2_q2_results.mat', ...
    'Ac','Bc','Cc','Dc','A','B','C','D','ts', ...
    'CMc','CMd','F','ps','pz','closed_loop_poles', ...
    'time','x','y','u','u_requested','theta_ref', ...
    'overshoot_percent','settling_time', ...
    'final_error_rad','peak_alpha_deg','peak_voltage', ...
    'saturation_time');