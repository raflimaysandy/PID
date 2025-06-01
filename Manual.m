% Parameter motor
R = 10;          % Ohm
L = 0.0002;        % Henry
J = 3e-7;        % kg.m^2
B = 2e-6;        % Nm.s/rad
Kt = 0.003;       % Nm/A
Ke = 0.003;       % V.s/rad

% Transfer function G(s) = ω(s)/V(s)
num = (Kt);
den = [L*J, (L*B + R*J), (R*B + Ke*Kt)];
G = tf(num, den);