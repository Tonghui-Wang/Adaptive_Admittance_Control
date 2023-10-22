function [x_r, dx_r, ddx_r]  = Impedence(x_d, dx_d, ddx_d, x_r, dx_r, f, dt, param)

% x_d：n*t时刻 广义期望位移 x desire
% dx_d：n*t时刻 广义期望速度 dx desire
% ddx_d：n*t时刻 广义期望加速度 ddx desire
% x_r：n*t时刻 广义实际位移 x reality
% dx_r：n*t时刻 广义实际速度 dx reality

% 阻抗模型f = m*(ddx_d-ddx_r) + b*(dx_d-dx_r) + k*(x_d-x_r)
m=param(1);
b=param(2);
k=param(3);
% m=1; %质量 kg
% b=5; %阻尼 N/m
% k=10; %刚度 N*s/m
% m=[1,1,1,1,1,1];
% b=[10,10,10,10,10,10];
% k=[10,10,10,10,10,10];

ddx_r = ddx_d + (b*(dx_d-dx_r) + k*(x_d-x_r) - f)/m; %n*t时刻
x_r = x_r + dx_r * dt;   %(n+1)*t时刻
dx_r = dx_r + ddx_r * dt;    %(n+1)*t时刻

end
