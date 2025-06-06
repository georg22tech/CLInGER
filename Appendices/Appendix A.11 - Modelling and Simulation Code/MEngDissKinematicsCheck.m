%syms s;
s = 1;
kappa = 0;
T = [cos(kappa*s) 0 sin(kappa*s) ((1 - cos(kappa*s))/kappa);
    0 1 0 0;
    -sin(kappa*s) 0 cos(kappa*s) (sin(kappa*s)/kappa);
    0 0 0 1];
%set initial position
phi = 0;
d = 50;
t = [0; 0; 1];
n = [cos(phi); sin(phi); 0];
b = cross(t, n);
%initialise joint variables
l_0 = [310 310];
step_delta_l = 3.14*2*5/10;
l = l_0;
p = zeros(3, 151);
p_t = zeros(4);
for i=1:1:150
    p_t(1) = p(1, i);
    p_t(2) = p(2, i);
    p_t(3) = p(3, i);
    p_t(4) = 1;
    l = l - [step_delta_l -1*step_delta_l];
    kappa = abs((l_0(2) - l(2))/(d*l(2)));
    T = [cos(kappa*s) 0 sin(kappa*s) ((1 - cos(kappa*s))/kappa);
    0 1 0 0;
    -sin(kappa*s) 0 cos(kappa*s) (sin(kappa*s)/kappa);
    0 0 0 1];
    p_t = T*p_t
    p(1, i+1) = p_t(1)
    p(2, i+1) = p_t(2)
    p(3, i+1) = p_t(3)
    %plot3 (p(1), p(2), p(3));
end
x = p(1, :);
y = p(3, :);
figure
scatter(x, y);
hold on
%Continuum
R = 100;
th = linspace( pi/2, pi, 100);
plot(R*cos(th)+100, R*sin(th))
ylim([0, 120])
xlim([0,120])
plot(R/th*cos(R^2/th)+100, R/th*sin(R^2/th))
ylim([0, 120])
xlim([0,120])
legend ("Kinematic output", "Constant Curvature Assumption", "Euler Curve Assumption")
hold off