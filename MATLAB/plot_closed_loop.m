function plot_closed_loop(time, u, x, x_hat, theta_ref, case_name, run_observer, save_figures)
% Closed-loop plots for the QUBE-Servo 2 RIP (DP2 Q2.5 / Q3.4).
%   x, x_hat : [theta alpha theta_dot alpha_dot] columns [rad, rad/s]
%   case_name: 'Linear' or 'Nonlinear'

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'Report', 'assets', 'images');
tag = lower(case_name);
if run_observer, tag = [tag '_obs']; else, tag = [tag '_sfc']; end

c_true = [0 0.4470 0.7410];  c_est = [0.8500 0.3250 0.0980];
c_ref = [0.25 0.25 0.25];    c_u = [0.4660 0.6740 0.1880];
r = rad2deg(theta_ref*(time >= 1));
th = rad2deg(x(:,1));  al = rad2deg(x(:,2));

%% Outputs and control effort
f1 = figure('Color', 'w', 'Position', [100 100 800 700]);
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile
plot(time, r, '--', 'Color', c_ref, 'LineWidth', 1.2); hold on
plot(time, th, 'Color', c_true, 'LineWidth', 1.5);
yline(rad2deg(theta_ref)*[0.98 1.02], ':', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
grid on; ylabel('\theta(t) [deg]');
legend('\theta^*', '\theta', 'Location', 'southeast');
title(sprintf('%s model', case_name));
nexttile
plot(time, zeros(size(time)), '--', 'Color', c_ref, 'LineWidth', 1.2); hold on
plot(time, al, 'Color', c_est, 'LineWidth', 1.5);
grid on; ylabel('\alpha(t) [deg]');
legend('\alpha^*', '\alpha', 'Location', 'northeast');
nexttile
plot(time, u, 'Color', c_u, 'LineWidth', 1.5);
grid on; ylabel('V_m(t) [V]'); xlabel('Time [s]');
legend('V_m', 'Location', 'northeast');
save_fig(f1, fullfile(out_dir, ['fig_' tag '_response.png']), save_figures);
print_metrics(time, u, x, x_hat, theta_ref, case_name, run_observer);

if ~run_observer, return; end

%% True vs estimated states, full run and zoom on the first 0.1 s
names = {'\theta', '\alpha', '\dot{\theta}', '\dot{\alpha}'};
hats  = {'\hat{\theta}', '\hat{\alpha}', '\dot{\hat{\theta}}', '\dot{\hat{\alpha}}'};
units = {'[deg]','[deg]', '[deg/s]', '[deg/s]'};
f2 = figure('Color', 'w', 'Position', [100 100 1000 800]);
tiledlayout(4, 2, 'TileSpacing', 'compact');
zoom_idx = time <= 0.1;
for i = 1:4
    for z = 0:1
        nexttile
        idx = true(size(time)); if z, idx = zoom_idx; end
        plot(time(idx), rad2deg(x(idx,i)), 'Color', c_true, 'LineWidth', 1.5); hold on
        plot(time(idx), rad2deg(x_hat(idx,i)), '--', 'Color', c_est, 'LineWidth', 1.5);
        grid on
        ylabel(['$' names{i} '(t)$ ' units{i}], 'Interpreter', 'latex');
        legend({['$' names{i} '$'], ['$' hats{i} '$']}, 'Interpreter', 'latex', 'Location', 'best');
        if i == 1
            if z, title('Zoom: first 0.1 s'); else, title(sprintf('%s model', case_name)); end
        end
        if i == 4, xlabel('Time [s]'); end
    end
end
save_fig(f2, fullfile(out_dir, ['fig_' tag '_estimation.png']), save_figures);

%% Estimation error
f3 = figure('Color', 'w', 'Position', [100 100 800 400]);
e = rad2deg(x - x_hat);
cols = lines(4);
for i = 1:4
    semilogy(time, abs(e(:,i)) + eps, 'Color', cols(i,:), 'LineWidth', 1.5); hold on
end
grid on; xlim([0 0.2]); ylim([1e-8 1e4]);
xlabel('Time [s]'); ylabel('$|e_i(t)|$ [deg], [deg/s]', 'Interpreter', 'latex');
legend({'$|\theta-\hat{\theta}|$', '$|\alpha-\hat{\alpha}|$', ...
        '$|\dot{\theta}-\dot{\hat{\theta}}|$', '$|\dot{\alpha}-\dot{\hat{\alpha}}|$'}, ...
       'Interpreter', 'latex', 'Location', 'northeast');
title(sprintf('%s model: observer estimation error', case_name));
save_fig(f3, fullfile(out_dir, ['fig_' tag '_error.png']), save_figures);
end

function print_metrics(time, u, x, x_hat, theta_ref, case_name, run_observer)
% Step metrics measured from the reference step at t = 1 s (2 % band)
k = time >= 1;  t = time(k) - 1;  th = x(k,1);
OS  = 100*(max(th) - theta_ref)/theta_ref;
out = find(abs(th - theta_ref) > 0.02*theta_ref, 1, 'last');
ts2 = t(min(out + 1, numel(t)));
ess = rad2deg(abs(th(end) - theta_ref));
fprintf('[%s, observer=%d] OS = %.2f %%, ts(2%%) = %.3f s, ess = %.2e deg, max|alpha| = %.2f deg, max|Vm| = %.2f V, Vm saturated for %.1f ms\n', ...
    case_name, run_observer, OS, ts2, ess, rad2deg(max(abs(x(:,2)))), max(abs(u)), ...
    1e3*sum(diff(time).*(abs(u(1:end-1)) >= 10 - 1e-9)));
if run_observer
    e = rad2deg(abs(x - x_hat));
    bad = find(max(e(:,1:2), [], 2) > 0.1, 1, 'last');
    fprintf('    angle estimation error < 0.1 deg after %.1f ms; final max|e| = %.2e\n', 1e3*time(bad + 1), max(e(end,:)));
end
end

function save_fig(f, path, do_save)
if do_save
    exportgraphics(f, path, 'Resolution', 200, 'BackgroundColor', 'white');
end
end
