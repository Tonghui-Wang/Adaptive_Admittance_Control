function [gbest]=Apso(x_di, dx_di, ddx_di, ...
                      x_ri, dx_ri, f, dt, ...
                      x_dii, dx_dii, ddx_dii)
%x_di dx_di ddx_di：i时刻期望位移 速度 加速度
%x_ri dx_ri：i时刻实际位移 速度
%f：i时刻各关节力矩偏差
%dt：采样间隔
%x_dii dx_dii：i+1时刻实际位移 速度
%返回的gbest是导纳最优参数mbk

%% 初始化
step=20;%进化次数
n=20;%种群规模
m=3;%粒子维度,即目标函数的自变量个数
xlimit=[0.1,0.1,0.1; 50,100,500];%目标函数自变量的取值范围
vmax=[1,2,3];%最大进化速度
cost=inf(n,1);%个体适应度代价
gbest=zeros(1,m);%群体最值
pbest=zeros(n,m);%个体最值
particle=zeros(2,m,n); %n个粒子，粒子每行分别表示位置、速度，每列对应一个自变量的迭代值
for i =1:n
    particle(1,:,i)=rand(1,m).*(xlimit(2,:)-xlimit(1,:)) + xlimit(1,:);%位置
    particle(2,:,i)=rand(1,m).*vmax;%速度
    % particle(3,:,i)=zeros(1,m);%结果，即当前目标函数自变量取值对应的因变量
    % pbest(i,:)=particle(1,:,i);
end

%step迭代
for k=1:step
    %% 适应度评价
    for i=1:n%循环n个粒子
        [shi,vel,acc]=Impedence(x_di, dx_di, ddx_di, x_ri, dx_ri, f, dt, particle(1,:,i));%(i+1)t时刻的实际轨迹方案
        error=norm(shi-x_dii)+norm(vel-dx_dii);%计算粒子当前代价
        if error < cost(i)%粒子当前代价小于粒子最小代价
            cost(i)=error;%更新粒子最小代价
            pbest(i,:)=particle(1,:,i);%更新粒子最优位置
        end
    end
    [costmin,i]=min(cost);%找到最小代价及索引
    gbest=pbest(i,:);%将最小代价的个体最优位置赋值给群体最优位置
    
    %% 个体进化
    c1=1.5;%自身认知系数
    c2=1.5;%群体认知系数
    wlimit=[0.4, 0.9];%惯性权重的取值范围
    costavg=mean(cost);%代价平均值
    
    for i=1:n
		%自适应惯性权重计算
        if cost(i)>costavg
            w=wlimit(2);
        elseif costavg-costmin==0
            w=wlimit(1)+rand*(wlimit(2)-wlimit(1));
        else
            w=wlimit(1)+(wlimit(2)-wlimit(1))*(cost(i)-costmin)/(costavg-costmin);
        end
        
        particle(2,:,i)=w*particle(2,:,i) + c1*rand(1,3).*pbest(i,:) + c2*rand(1,3).*gbest;%个体速度进化
        particle(2,:,i)=min(particle(2,:,i), vmax);%最大速度进化约束
        particle(1,:,i)=particle(1,:,i) + particle(2,:,i);%个体位置进化
        particle(1,:,i)=max(particle(1,:,i), xlimit(1,:));%最小位置进化约束
        particle(1,:,i)=min(particle(1,:,i), xlimit(2,:));%最大速度进化约束
        % particle(3,:,i)=zeros(1,m);%个体结果复位
    end
end
end
