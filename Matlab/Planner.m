function [q, qd, qdd] = Planner(viaDeg, dt)
%Joint-space multi-segment trajectory.
%Central differences are used at the ends as well, so the last sample is
%not differenced against an implicit zero.
via = deg2rad(viaDeg);
qdmax = 0.5 * ones(1, 6);
q = mstraj(via, qdmax, [], [], dt, 0.5);
n = size(q, 1);
qd = zeros(n, 6);
qdd = zeros(n, 6);
qd(1, :) = (q(2, :) - q(1, :)) / dt;
qd(n, :) = (q(n, :) - q(n - 1, :)) / dt;
for i = 2:n - 1
    qd(i, :) = (q(i + 1, :) - q(i - 1, :)) / (2 * dt);
end
qdd(1, :) = (qd(2, :) - qd(1, :)) / dt;
qdd(n, :) = (qd(n, :) - qd(n - 1, :)) / dt;
for i = 2:n - 1
    qdd(i, :) = (qd(i + 1, :) - qd(i - 1, :)) / (2 * dt);
end
end
