clear;
clc;

%% 1. MDH建模
a = [0, 150, 750, 155, 0, 0]/1000;
alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2];
d = [253, 0, 0, 800, 0, 154]/1000;
theta = [0, pi/2, 0, 0, -pi/2, 0];

ecr5=Model(a,alpha,d,theta,'ecr5');

clear a alpha d theta;

%% 2. 期望轨迹规划
jpoints=[25, 20, 30, 40, 50, 60;
    -5, -10, -15, -20, -25, -30;
    -45, -20, 25, -5, 35, -50];
dt=0.01;

[q_d, dq_d, ddq_d] = Planner(jpoints, dt);

clear jpoints;

%% 3. 实际轨迹导纳
[n,m]=size(q_d);
q_r=zeros(n,m);
dq_r=zeros(n,m);
ddq_r=zeros(n,m);
q_r(1,:)=q_d(1,:);
dq_r(1,:)=dq_d(1,:);
ddq_r(1,:)=ddq_d(1,:);

f=zeros(n,m);%末端力
x=linspace(0,10,100);
x=[x,fliplr(x)];
j=101;
for i = j:j+length(x)-1
    f(i,:)=[0, -x(i-j+1), x(i-j+1), 0, x(i-j+1), 0];
end

for i = 1:n-1
    %根据理论坐标计算雅可比矩阵，再根据雅可比矩阵和末端外力计算关节力矩偏差
    tau=ecr5.jacob0(q_d(i,:))'*f(i,:)';
    
    %通过粒子群算法，求取导纳控制的最优参数，实现自适应效果
    param = Apso(q_d(i,:), dq_d(i,:), ddq_d(i,:), ...
                q_r(i,:), dq_r(i,:), tau', dt, ...
                q_d(i+1,:), dq_d(i+1,:), ddq_d(i+1,:));

    %通过导纳控制，求取下一采样时刻的实际轨迹
    [shi,vel,acc]=Impedence(q_d(i,:), dq_d(i,:), ddq_d(i,:), ...
                     q_r(i,:), dq_r(i,:), tau', dt, param);

    q_r(i+1,:)=shi;
    dq_r(i+1,:)=vel;
    ddq_r(i,:)=acc;
end

clear m f x i j dt tau param shi vel acc;

%% 4. 柔顺效果展示
p_d=zeros(n,3);
p_r=zeros(n,3);
for i=1:n;
    p_d(i,:)=ecr5.fkine(q_d(i,:)).t';
    p_r(i,:)=ecr5.fkine(q_r(i,:)).t';
end
figure(1);
hold on;
title("末端轨迹");
plot3(p_d(:,1),p_d(:,2),p_d(:,3),'r','Linewidth',3)%, p_r(:,1),p_r(:,2),p_r(:,3),'b');
ecr5.plot(q_r,'trail','b');

figure(2);
hold on;
sgtitle("关节位移");
i=1:n;
subplot(611)
plot(i,q_d(:,1),'r', i,q_r(:,1),'b');
subplot(612)
plot(i,q_d(:,2),'r', i,q_r(:,2),'b');
subplot(613)
plot(i,q_d(:,3),'r', i,q_r(:,3),'b');
subplot(614)
plot(i,q_d(:,4),'r', i,q_r(:,4),'b');
subplot(615)
plot(i,q_d(:,5),'r', i,q_r(:,5),'b');
subplot(616)
plot(i,q_d(:,6),'r', i,q_r(:,6),'b');

figure(3);
hold on;
sgtitle("关节速度");
subplot(611)
plot(i,dq_d(:,1),'r', i,dq_r(:,1),'b');
subplot(612)
plot(i,dq_d(:,2),'r', i,dq_r(:,2),'b');
subplot(613)
plot(i,dq_d(:,3),'r', i,dq_r(:,3),'b');
subplot(614)
plot(i,dq_d(:,4),'r', i,dq_r(:,4),'b');
subplot(615)
plot(i,dq_d(:,5),'r', i,dq_r(:,5),'b');
subplot(616)
plot(i,dq_d(:,6),'r', i,dq_r(:,6),'b');

figure(4);
hold on;
sgtitle("关节加速度");
subplot(611)
plot(i,ddq_d(:,1),'r', i,ddq_r(:,1),'b');
subplot(612)
plot(i,ddq_d(:,2),'r', i,ddq_r(:,2),'b');
subplot(613)
plot(i,ddq_d(:,3),'r', i,ddq_r(:,3),'b');
subplot(614)
plot(i,ddq_d(:,4),'r', i,ddq_r(:,4),'b');
subplot(615)
plot(i,ddq_d(:,5),'r', i,ddq_r(:,5),'b');
subplot(616)
plot(i,ddq_d(:,6),'r', i,ddq_r(:,6),'b');

clear i n;
