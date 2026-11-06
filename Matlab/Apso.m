function [best, bestCost] = Apso(costfun, x0, lb, ub, vmax, particles, iterations)
%Standard particle swarm. Velocity points at (pbest-x) and (gbest-x),
%and a particle that hits a bound loses its outward velocity.
x0 = x0(:).';
dim = numel(x0);
lb = lb(:).';
ub = ub(:).';
vmax = vmax(:).';
pos = repmat(x0, particles, 1);
span = (ub - lb) * 0.12;
pos(2:end, :) = pos(2:end, :) + randn(particles - 1, dim) .* span;
pos = min(max(pos, lb), ub);
vel = (2 * rand(particles, dim) - 1) .* (vmax * 0.2);
personal = pos;
pcost = inf(particles, 1);
for i = 1:particles
    pcost(i) = costfun(pos(i, :));
end
[gcost, index] = min(pcost);
gbest = personal(index, :);
for generation = 0:iterations - 1
    inertia = 0.9 - 0.5 * generation / max(iterations - 1, 1);
    for i = 1:particles
        vel(i, :) = inertia * vel(i, :) ...
            + 1.5 * rand(1, dim) .* (personal(i, :) - pos(i, :)) ...
            + 1.5 * rand(1, dim) .* (gbest - pos(i, :));
        vel(i, :) = min(max(vel(i, :), -vmax), vmax);
        pos(i, :) = pos(i, :) + vel(i, :);
        outside = pos(i, :) <= lb | pos(i, :) >= ub;
        pos(i, :) = min(max(pos(i, :), lb), ub);
        vel(i, outside) = 0;
        value = costfun(pos(i, :));
        if ~isfinite(value)
            value = 1e12;
        end
        if value < pcost(i)
            pcost(i) = value;
            personal(i, :) = pos(i, :);
            if value < gcost
                gcost = value;
                gbest = pos(i, :);
            end
        end
    end
end
best = gbest;
bestCost = gcost;
end
