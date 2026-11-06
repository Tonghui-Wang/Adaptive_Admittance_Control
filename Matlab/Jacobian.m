function J = Jacobian(q)
%Geometric Jacobian in the base frame, metres and radians.
%Modified DH: joint i rotates about the z axis of frame i.
[~, frames] = Fkine(q);
origin = frames{end}(1:3, 4);
J = zeros(6, 6);
for i = 1:6
    axis = frames{i + 1}(1:3, 3);
    point = frames{i + 1}(1:3, 4);
    J(1:3, i) = cross(axis, origin - point);
    J(4:6, i) = axis;
end
end
