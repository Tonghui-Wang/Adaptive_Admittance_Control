function Main()
%ECR5 constant-force contour with selective Cartesian admittance.
%Requires Peter Corke's Robotics Toolbox (SerialLink, RevoluteMDH, mstraj).
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);
c = Params();

via = [10, 20, 30, 40, 50, 60; -5, -10, -15, -20, -25, -30; 35, -20, 25, -30, 35, -50];
a = [0, 150, 750, 155, 0, 0] / 1000;
alpha = [0, pi / 2, 0, pi / 2, -pi / 2, pi / 2];
d = [253, 0, 0, 800, 0, 154] / 1000;
theta = [0, pi / 2, 0, 0, -pi / 2, 0];
robot = Model(a, alpha, d, theta, 'ECR5');

[q, qd, qdd] = Planner(via, c.dt);
count = size(q, 1);
position = zeros(count, 3);
speed = zeros(count, 3);
for i = 1:count
    pose = robot.fkine(q(i, :));
    position(i, :) = pose.t(:).';
    jacobian = robot.jacob0(q(i, :));
    speed(i, :) = (jacobian(1:3, :) * qd(i, :).').';
end
ref.t = (0:count - 1).' * c.dt;
ref.q = q;
ref.qd = qd;
ref.qdd = qdd;
ref.p = position;
ref.v = speed;
ref.a = pathAccel(speed, c.dt);
[normal, planePoint] = fitPlane(position);

modes = {'adaptive', 'fixed', 'heavy', 'light'};
runs = struct();
for k = 1:numel(modes)
    if strcmp(modes{k}, 'adaptive')
        rng(7);
    end
    runs.(modes{k}) = Simulate(robot, ref, normal, planePoint, modes{k});
end

switchTime = 0.5 * ref.t(end);
outDir = fullfile(thisDir, '..', 'output');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
save(fullfile(outDir, 'matlab_runs.mat'), 'runs', 'switchTime', 'normal', 'planePoint');
plotResults(runs, switchTime, normal, planePoint, outDir);
printMetrics(runs, switchTime);
end

function accel = pathAccel(speed, dt)
count = size(speed, 1);
accel = zeros(size(speed));
accel(1, :) = (speed(2, :) - speed(1, :)) / dt;
accel(count, :) = (speed(count, :) - speed(count - 1, :)) / dt;
for i = 2:count - 1
    accel(i, :) = (speed(i + 1, :) - speed(i - 1, :)) / (2 * dt);
end
end

function [normal, point] = fitPlane(points)
center = mean(points, 1);
[axes, values] = eig(cov(points));
[~, order] = sort(diag(values));
normal = axes(:, order(1)).';
normal = normal / norm(normal);
offset = (points - center) * normal.';
if offset(1) < median(offset)
    normal = -normal;
    offset = -offset;
end
point = center + normal * (offset(1) - 0.015);
end

function printMetrics(runs, switchTime)
modes = {'adaptive', 'fixed', 'heavy', 'light'};
names = {'自适应', '固定中等', '固定偏重', '固定偏轻'};
fprintf('刚度跳变时刻 %.2f s\n', switchTime);
for k = 1:numel(modes)
    result = runs.(modes{k});
    steady = windowRmse(result, 1.0, switchTime);
    recover = windowRmse(result, switchTime, switchTime + 1.0);
    tangent = 1000 * sqrt(mean(result.tangent .^ 2));
    fprintf('%s  稳态 %.3f N  跳变后 %.3f N  切向 %.2f mm\n', names{k}, steady, recover, tangent);
end
end

function value = windowRmse(result, t0, t1)
mask = result.t >= t0 & result.t < t1;
error = result.force(mask) - result.forceDes(mask);
value = sqrt(mean(error .^ 2));
end

function plotResults(runs, switchTime, normal, planePoint, outDir)
modes = {'adaptive', 'fixed', 'heavy', 'light'};
names = {'自适应', '固定中等', '固定偏重', '固定偏轻'};
colors = [0.04, 0.43, 0.31; 0.12, 0.31, 0.47; 0.72, 0.36, 0.22; 0.48, 0.42, 0.60];

figure(1);
clf;
hold on;
plot(runs.adaptive.t, runs.adaptive.forceDes, 'k--', 'LineWidth', 1.2);
for k = 1:numel(modes)
    plot(runs.(modes{k}).t, runs.(modes{k}).force, 'Color', colors(k, :), 'LineWidth', 1.2);
end
xline(switchTime, ':');
xlabel('时间 (s)');
ylabel('法向接触力 (N)');
title('接触力');
legend([{'期望力'}, names], 'Location', 'northeast');
grid on;
saveas(gcf, fullfile(outDir, 'matlab_force.png'));

figure(2);
clf;
adaptive = runs.adaptive;
subplot(3, 1, 1);
plot(adaptive.t, adaptive.m, 'Color', colors(1, :));
ylabel('m (kg)');
xline(switchTime, ':');
title('法向导纳参数与刚度估计');
subplot(3, 1, 2);
plot(adaptive.t, adaptive.b, 'Color', colors(1, :));
ylabel('b (N·s/m)');
xline(switchTime, ':');
subplot(3, 1, 3);
plot(adaptive.t, adaptive.kTrue, '--', 'Color', [0.5, 0.5, 0.5]);
hold on;
plot(adaptive.t, adaptive.kHat, 'Color', colors(1, :));
ylabel('刚度 (N/m)');
xlabel('时间 (s)');
xline(switchTime, ':');
saveas(gcf, fullfile(outDir, 'matlab_gains.png'));

figure(3);
clf;
hold on;
plot3(adaptive.pRef(:, 1), adaptive.pRef(:, 2), adaptive.pRef(:, 3), '--', 'Color', colors(3, :));
plot3(adaptive.position(:, 1), adaptive.position(:, 2), adaptive.position(:, 3), 'Color', colors(1, :));
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('末端路径');
grid on;
legend('期望路径', '实际路径');
saveas(gcf, fullfile(outDir, 'matlab_path.png'));
end
