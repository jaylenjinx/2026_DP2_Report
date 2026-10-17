%% Physical parameters
Rm = 8.4;
kt = 0.042;
km = 0.042;

mr = 0.095;
Lr = 0.085;
mp = 0.024;
Lp = 0.129;

Jm = 4e-6;
Jh = 0.6e-6;
Dr = 0.0015;
Dp = 0.0005;
g = 9.81;

%% Derived constants
lp = Lp/2;
Jr = mr*Lr^2/3 + Jm + Jh;
Jp = mp*Lp^2/12;

a = Jr + mp*Lr^2;
b = Jp + mp*lp^2;
c = mp*Lr*lp;
h = mp*g*lp;
d = Dr + kt*km/Rm;
kv = kt/Rm;
Delta0 = a*b - c^2;

%% Continuous-time model
Ac = [0, 0, 1, 0;
      0, 0, 0, 1;
      0, c*h/Delta0, -b*d/Delta0, -c*Dp/Delta0;
      0, a*h/Delta0, -c*d/Delta0, -a*Dp/Delta0];

Bc = [0; 0; b*kv/Delta0; c*kv/Delta0];

Cc = [1 0 0 0;
      0 1 0 0];

Dc = zeros(2,1);

%% Discretise using a zero-order hold
ts = 0.001;
sys_ct = ss(Ac, Bc, Cc, Dc);
sys_dt = c2d(sys_ct, ts, 'zoh');

format long g
A = sys_dt.A
B = sys_dt.B
C = sys_dt.C
D = sys_dt.D