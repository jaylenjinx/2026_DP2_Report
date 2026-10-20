%% Plotting closed-loop response (nonlinear model)
% Signals exported from Simulink: time, u, x_NL, y_NL, x_hat
plot_closed_loop(time, u(:,1), x_NL, x_hat, theta_ref, 'Nonlinear', run_observer, save_figures);
