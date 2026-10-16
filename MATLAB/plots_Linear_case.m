%% Plotting Open- or Closed-loop response
% These are examples to generate some plots
% You have to modify them or create new ones

y1 = y_L(:,1);
y2 = y_L(:,2);

u1 = u(:,1);

x1 = x_L(:,1);
x2 = x_L(:,2);

x1_hat = x_hat(:,1);
x2_hat = x_hat(:,2);


figure(11)
subplot(311)
plot(time,u1)
title('Linear case','fontsize',16,'interpreter','latex')

subplot(312)
plot(time,y1)

subplot(313)
plot(time,y2)


figure(12)
subplot(311)
plot(time,u1)

subplot(312)
plot(time,x1,time,x1_hat,'r','LineWidth',1.2)

subplot(313)
plot(time,x2,time,x2_hat,'r','LineWidth',1.2)

