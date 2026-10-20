%% Plotting closed-loop response (linear model)
% Signals exported from Simulink: time, u, x_L, y_L, x_hat
plot_closed_loop(time, u(:,1), x_L, x_hat, theta_ref, 'Linear', run_observer, save_figures);
