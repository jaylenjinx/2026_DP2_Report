function x_dot = rip_nonlinear_dynamics(x, u, P)
% Nonlinear rotary inverted pendulum (QUBE-Servo 2), alpha = 0 upright.
%   x = [theta; alpha; theta_dot; alpha_dot]   [rad, rad, rad/s, rad/s]
%   u = motor voltage Vm                        [V]
%   P = [a b c h mp*lp^2 Dr Dp kt km Rm]        (see dp2_q1.m)
% Linearising about x = 0 gives Ac, Bc in dp2_q1.m.

a   = P(1);  b  = P(2);  c  = P(3);  h  = P(4);  mlp2 = P(5);
Dr  = P(6);  Dp = P(7);  kt = P(8);  km = P(9);  Rm   = P(10);

th_d = x(3);  al = x(2);  al_d = x(4);
s = sin(al);  co = cos(al);

tau = kt*(u - km*th_d)/Rm;   % motor torque from armature circuit (Lm neglected)

M = [a + mlp2*s^2,  -c*co;
     -c*co,          b    ];
f = [tau - Dr*th_d - 2*mlp2*s*co*th_d*al_d - c*s*al_d^2;
     -Dp*al_d + mlp2*s*co*th_d^2 + h*s];

q_dd = M\f;
x_dot = [th_d; al_d; q_dd];
end
