function [J] = Jacobian(q)

n = length(q); % number of degrees of freedom

% preallocationg the Jacobian matrix
jv = zeros(3, n);
jw = zeros(3, n);

k = [0; 0; 1];
Ts = Tmatrixs(q);

for i = 1: n
   r = Ts{i}(1: 3, 1: 3);
   t = (inv(Ts{i})) * Ts{n};
   p = t(1: 3, 4);
   z = r * k;
   
   jv(:, i) = cross(z, r*p);
   jw(:, i) = z;
end

J = [jv; jw];
end

function [Ts] = Tmatrixs(q)

n = length(q);

a1=150;
a2=750;
a3=155;
d1=253;
d4=800;
d6=154;

a = [0, a1, a2, a3, 0, 0];
alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2];
d = [d1, 0, 0, d4, 0, d6];
theta = [0, pi/2, 0, 0, -pi/2, 0];
theta = theta + deg2rad(q);

t = eye(4);
% Ts{n} = zeros(4);
for i = 1 : n
    A = [cos(theta(i)),             -sin(theta(i)),               0,            a(i);
        sin(theta(i))*cos(alpha(i)), cos(theta(i))*cos(alpha(i)), -sin(alpha(i)), -sin(alpha(i))*d(i);
        sin(theta(i))*sin(alpha(i)), cos(theta(i))*sin(alpha(i)), cos(alpha(i)),  cos(alpha(i))*d(i);
        0,            0,            0,            1];
    t = t * A;
    Ts{i} = t;
end
end
