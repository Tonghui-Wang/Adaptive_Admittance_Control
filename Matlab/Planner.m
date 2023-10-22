function [x_d,dx_d,ddx_d] = Planner(point, dt)

segments=deg2rad(point);%途经点
qdmax=[0.3, 0.4, 0.5, 0.6, 0.4, 0.3];%最大速度
tsegment=[];%每段运行时间
q0=[];%起点各轴坐标
tacc=0.5;%抛物线过渡时间

x_d= mstraj(segments, qdmax, tsegment, q0, dt, tacc);

[n,m]=size(x_d);
dx_d=zeros(n,m);
ddx_d=zeros(n,m);
for i =1:n-1
    dx_d(i,:)=(x_d(i+1,:)-x_d(i,:))/dt;
end
for i =1:n-1
    ddx_d(i,:)=(dx_d(i+1,:)-dx_d(i,:))/dt;
end
end
