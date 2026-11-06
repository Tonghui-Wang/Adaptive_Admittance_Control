function [T, frames] = Fkine(q)
%Forward kinematics of ECR5 with modified DH.
%q is 1x6 in radians. T is 4x4 in metres. frames{1} is the base.
a = [0, 0.150, 0.750, 0.155, 0, 0];
alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2];
d = [0.253, 0, 0, 0.800, 0, 0.154];
offset = [0, pi / 2, 0, 0, -pi / 2, 0];
T = eye(4);
frames = cell(1, 7);
frames{1} = T;
for i = 1:6
    T = T * mdh(a(i), alpha(i), d(i), q(i) + offset(i));
    frames{i + 1} = T;
end
end

function A = mdh(a, alpha, d, theta)
ca = cos(alpha);
sa = sin(alpha);
ct = cos(theta);
st = sin(theta);
A = [ct, -st, 0, a;
     st * ca, ct * ca, -sa, -sa * d;
     st * sa, ct * sa, ca, ca * d;
     0, 0, 0, 1];
end
