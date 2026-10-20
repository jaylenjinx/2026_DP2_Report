%% Plot linear Simulink results
% Supports Array or Timeseries logging.

%% Retrieve results
if exist('out','var') && isa(out,'Simulink.SimulationOutput')
    y_L = out.get('y_L');
    u = out.get('u');
    time = out.get('time');
end

%% Extract outputs
if isa(y_L,'timeseries')
    t_y = y_L.Time(:);
    y_data = squeeze(y_L.Data);

    if ~y_L.IsTimeFirst
        y_data = y_data.';
    end
else
    y_data = squeeze(y_L);

    if isa(time,'timeseries')
        t_y = time.Data(:);
    else
        t_y = time(:);
    end

    if size(y_data,1) ~= numel(t_y) && ...
            size(y_data,2) == numel(t_y)
        y_data = y_data.';
    end
end

%% Extract voltage
if isa(u,'timeseries')
    t_u = u.Time(:);
    u_data = squeeze(u.Data);
else
    t_u = t_y;
    u_data = squeeze(u);
end

u_data = u_data(:);

assert(size(y_data,1)==numel(t_y) && size(y_data,2)==2, ...
    'Expected y_L to contain one row per sample and two angles.');

assert(numel(u_data)==numel(t_u), ...
    'Voltage and time have different lengths. Check logging sample times.');

%% Reference: 5-degree step at 1 second
reference_deg = 5*double(t_y >= 1);

%% Plot results
fig = figure(89);
clf
set(fig,'Color','w','Position',[100 100 1000 800]);

subplot(3,1,1)
plot(t_y,y_data(:,1)*180/pi,'b','LineWidth',1.2)
hold on
plot(t_y,reference_deg,'r--','LineWidth',1.2)
ylabel('Arm angle (deg)')
title('Linear Simulink response')
legend('Arm angle','Reference','Location','best')
grid on
ylim padded

subplot(3,1,2)
plot(t_y,y_data(:,2)*180/pi,'LineWidth',1.2)
ylabel('Pendulum angle (deg)')
grid on
ylim padded

subplot(3,1,3)
plot(t_u,u_data,'LineWidth',1.2)
ylabel('Motor voltage (V)')
xlabel('Time (s)')
grid on
ylim padded

%% Save figure in the current MATLAB folder
drawnow
exportgraphics(fig,'Q2_Linear_Response.png','Resolution',300);
exportgraphics(fig,'Q2_Linear_Response.pdf','ContentType','vector');
savefig(fig,'Q2_Linear_Response.fig');

%% Performance measured from the Simulink results
command_rad = 5*pi/180;
step_time = 1;

idx = find(t_y >= step_time);
theta_sim = y_data(idx,1);

overshoot = max(0, ...
    100*(max(theta_sim)-command_rad)/abs(command_rad));

outside = find(abs(theta_sim-command_rad) ...
    > 0.02*abs(command_rad));

if isempty(outside)
    settling = 0;
elseif outside(end) == numel(idx)
    settling = NaN;
else
    settling = t_y(idx(outside(end)+1))-step_time;
end

fprintf('\Linear Simulink results:\n');
fprintf('Overshoot: %.3f %%\n',overshoot);
fprintf('2%% settling time after step: %.3f s\n',settling);
fprintf('Final arm error: %.6g deg\n', ...
    (command_rad-y_data(end,1))*180/pi);
fprintf('Peak absolute pendulum angle: %.4f deg\n', ...
    max(abs(y_data(:,2)))*180/pi);
fprintf('Peak absolute voltage: %.4f V\n', ...
    max(abs(u_data)));