function out = Simulate(robot, ref, normal, planePoint, mode)
%Closed-loop selective admittance on the virtual plane.
c = Params();
nvec = normal(:).';
plane = planePoint(:).';
switch mode
    case 'light'
        mass = 0.8;
        damp = 25;
    case 'heavy'
        mass = 5;
        damp = 280;
    otherwise
        mass = 1.5;
        damp = 80;
end
count = size(ref.q, 1);
q = ref.q(1, :).';
qdot = ref.qd(1, :).';
kHat = c.kHat0;
duration = ref.t(end);

out.force = zeros(count, 1);
out.forceDes = zeros(count, 1);
out.penetration = zeros(count, 1);
out.m = zeros(count, 1);
out.b = zeros(count, 1);
out.kHat = zeros(count, 1);
out.kTrue = zeros(count, 1);
out.tangent = zeros(count, 1);
out.position = zeros(count, 3);

for i = 1:count
    pose = robot.fkine(q.');
    position = pose.t(:).';
    jacobian = robot.jacob0(q.');
    velocity = (jacobian(1:3, :) * qdot).';
    kTrue = environmentStiffness(ref.t(i), duration, c);
    [force, penetration] = contactForce(position, nvec, plane, kTrue);
    forceStar = desiredForce(ref.t(i), c);
    if penetration > 3e-3
        measured = min(max(force / penetration, 100), 30000);
        if abs(measured - kHat) > 0.4 * kHat
            kHat = measured;
        else
            kHat = 0.8 * kHat + 0.2 * measured;
        end
    end

    if strcmp(mode, 'adaptive') && mod(i - 1, c.adaptEvery) == 0 && penetration > 1e-3
        current = [mass, damp];
        baseline = previewCost(current);
        [proposed, proposedCost] = Apso( ...
            @previewCost, current, c.mBounds, c.bBounds, c.vmax, c.particles, c.iters);
        if proposedCost + 1e-3 < baseline
            limited = current + max(min(proposed - current, c.rate), -c.rate);
            mass = limited(1);
            damp = limited(2);
        end
    end

    accel = Impedence( ...
        position, velocity, ref.p(i, :), ref.v(i, :), ref.a(i, :), ...
        nvec, force, forceStar, penetration, mass, damp);
    velocityCmd = integrateVelocity(velocity, accel, c, nvec, penetration);
    qdot = resolvedRate(jacobian(1:3, :), velocityCmd.', q, ref.q(i, :).', ref.qd(i, :).', c);
    q = q + qdot * c.dt;

    tangent = (position - ref.p(i, :)) - nvec * ((position - ref.p(i, :)) * nvec.');
    out.force(i) = force;
    out.forceDes(i) = forceStar;
    out.penetration(i) = penetration;
    out.m(i) = mass;
    out.b(i) = damp;
    out.kHat(i) = kHat;
    out.kTrue(i) = kTrue;
    out.tangent(i) = norm(tangent);
    out.position(i, :) = position;
end
out.t = ref.t;
out.pRef = ref.p;

    function value = previewCost(gains)
        point = position;
        speed = velocity;
        forceError = 0;
        tangentError = 0;
        for step = 0:c.horizon - 1
            index = min(max(i + step, 1), count);
            [forceTry, penTry] = contactForce(point, nvec, plane, kHat);
            forceTryDes = desiredForce(ref.t(index), c);
            accelTry = Impedence( ...
                point, speed, ref.p(index, :), ref.v(index, :), ref.a(index, :), ...
                nvec, forceTry, forceTryDes, penTry, gains(1), gains(2));
            speed = integrateVelocity(speed, accelTry, c, nvec, penTry);
            point = point + speed * c.dt;
            forceError = forceError + ((forceTry - forceTryDes) / c.fStar) ^ 2;
            tang = (point - ref.p(index, :)) - nvec * ((point - ref.p(index, :)) * nvec.');
            tangentError = tangentError + (tang * tang.') / (0.02 ^ 2);
        end
        smooth = (gains(:).' - [mass, damp]) ./ [1, 40];
        value = forceError / c.horizon + 0.05 * tangentError / c.horizon + 0.02 * (smooth * smooth.');
        if ~isfinite(value)
            value = 1e12;
        end
    end
end

function [force, penetration] = contactForce(position, normal, plane, stiffness)
signed = (position - plane) * normal.';
penetration = max(0, -signed);
force = stiffness * penetration;
end

function force = desiredForce(t, c)
force = c.fStar * min(max(t / c.fRamp, 0), 1);
end

function stiffness = environmentStiffness(t, duration, c)
if t < 0.5 * duration
    stiffness = c.kSoft;
else
    stiffness = c.kStiff;
end
end

function speed = integrateVelocity(velocity, accel, c, normal, penetration)
speed = velocity + accel * c.dt;
speedN = speed * normal.';
if penetration <= 1e-6 && speedN < -c.approach
    speed = speed + (-c.approach - speedN) * normal;
    speedN = -c.approach;
end
if speedN > 0.5
    speed = speed + (0.5 - speedN) * normal;
end
end

function qdot = resolvedRate(jacobianV, velocityCmd, q, qRef, qdRef, c)
weight = jacobianV * jacobianV.' + (c.lambda ^ 2) * eye(3);
jpinv = jacobianV.' / weight;
nullspace = eye(6) - jpinv * jacobianV;
qdot = jpinv * velocityCmd + nullspace * (qdRef + c.nullGain * (qRef - q));
qdot = min(max(qdot, -c.qdotLimit), c.qdotLimit);
end
