import numpy as np
from math import pi
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

def Main():

    # 1. MDH建模
    a = np.array([0, 150, 750, 155, 0, 0])/1000
    alpha = np.array([0, pi/2, 0, pi/2, -pi/2, pi/2])
    d = np.array([253, 0, 0, 800, 0, 154])/1000
    theta = np.array([0, pi/2, 0, 0, -pi/2, 0])

    ecr5 = Model(a=a, alpha=alpha, d=d, theta=theta)

    # 2. 期望轨迹规划
    jpoints=np.array([[10,20,30,40,50,60],
                    [-5,-10,-15,-20,-25,-30],
                    [35,-20,25,-30,35,-50]])    # 确定的关节途径点
    dt=0.01 # 任务周期，s
    
    q_d, dq_d, ddq_d = Planner(point=jpoints, dt=dt)    # 期望轨迹规划，位移、速度、加速度

    # 3. 实际轨迹导纳
    n,m = q_d.shape
    f=np.zeros((n,m))   #末端的各方向外力
    ff=np.linspace(0,10,50)
    ff=np.concatenate((ff,ff[::-1]))
    f[50:50+2*50,2]=ff

    q_r=np.zeros((n,m)) # 实际位移
    dq_r=np.zeros((n,m))    # 实际速度
    ddq_r=np.zeros((n,m))   # 实际加速度
    q_r[0,:]=q_d[0,:]
    dq_r[0,:]=dq_d[0,:]
    ddq_r[0,:]=ddq_d[0,:]

    for i in range(n-1):
        # 根据理论坐标计算雅可比矩阵，再根据雅可比矩阵和末端外力计算关节力矩偏差
        # tau=np.dot(ecr5.jacob0(q_d[i,:]).T, f[i,:].T)
        tau=np.dot(f[i,:], ecr5.jacob0(q_d[i,:]))

        # 通过粒子群算法，求取导纳控制的最优控制参数，实现自适应效果
        param=Apso(x_di=q_d[i,:], dx_di=dq_d[i,:], ddx_di=ddq_d[i,:],
                x_ri=q_r[i,:], dx_ri=dq_r[i,:], f=tau.T, dt=dt,
                x_dii=q_d[i+1,:], dx_dii=dq_d[i+1,:], ddx_dii=ddq_d[i+1,:])

        # 通过导纳控制，求取下一任务时刻的实际轨迹
        shi,vel,acc=Impedence(x_d=q_d[i,:], dx_d=dq_d[i,:], ddx_d=ddq_d[i,:], 
                        x_r=q_r[i,:], dx_r=dq_r[i,:], f=tau.T, dt=dt, param=param)

        q_r[i+1,:]=shi
        dq_r[i+1,:]=vel
        ddq_r[i,:]=acc

    # 4. 柔顺效果展示
    i=range(n)

    plt.figure(1)
    plt.suptitle('shift')
    plt.subplot(611)
    plt.plot(i,q_d[:,0],'r--', i,q_r[:,0],'b:', i,q_r[:,0]-q_d[:,0],'g-')
    plt.subplot(612)
    plt.plot(i,q_d[:,1],'r--', i,q_r[:,1],'b:', i,q_r[:,1]-q_d[:,1],'g-')
    plt.subplot(613)
    plt.plot(i,q_d[:,2],'r--', i,q_r[:,2],'b:', i,q_r[:,2]-q_d[:,2],'g-')
    plt.subplot(614)
    plt.plot(i,q_d[:,3],'r--', i,q_r[:,3],'b:', i,q_r[:,3]-q_d[:,3],'g-')
    plt.subplot(615)
    plt.plot(i,q_d[:,4],'r--', i,q_r[:,4],'b:', i,q_r[:,4]-q_d[:,4],'g-')
    plt.subplot(616)
    plt.plot(i,q_d[:,5],'r--', i,q_r[:,5],'b:', i,q_r[:,5]-q_d[:,5],'g-')

    plt.figure(2)
    plt.suptitle('velocity')
    plt.subplot(611)
    plt.plot(i,dq_d[:,0],'r--', i,dq_r[:,0],'b:', i,dq_r[:,0]-dq_d[:,0],'g-')
    plt.subplot(612)
    plt.plot(i,dq_d[:,1],'r--', i,dq_r[:,1],'b:', i,dq_r[:,1]-dq_d[:,1],'g-')
    plt.subplot(613)
    plt.plot(i,dq_d[:,2],'r--', i,dq_r[:,2],'b:', i,dq_r[:,2]-dq_d[:,2],'g-')
    plt.subplot(614)
    plt.plot(i,dq_d[:,3],'r--', i,dq_r[:,3],'b:', i,dq_r[:,3]-dq_d[:,3],'g-')
    plt.subplot(615)
    plt.plot(i,dq_d[:,4],'r--', i,dq_r[:,4],'b:', i,dq_r[:,4]-dq_d[:,4],'g-')
    plt.subplot(616)
    plt.plot(i,dq_d[:,5],'r--', i,dq_r[:,5],'b:', i,dq_r[:,5]-dq_d[:,5],'g-')

    plt.figure(3)
    plt.suptitle('accelerate')
    plt.subplot(611)
    plt.plot(i,ddq_d[:,0],'r--', i,ddq_r[:,0],'b:', i,ddq_r[:,0]-ddq_d[:,0],'g:')
    plt.subplot(612)
    plt.plot(i,ddq_d[:,1],'r--', i,ddq_r[:,1],'b:', i,ddq_r[:,1]-ddq_d[:,1],'g:')
    plt.subplot(613)
    plt.plot(i,ddq_d[:,2],'r--', i,ddq_r[:,2],'b:', i,ddq_r[:,2]-ddq_d[:,2],'g-')
    plt.subplot(614)
    plt.plot(i,ddq_d[:,3],'r--', i,ddq_r[:,3],'b:', i,ddq_r[:,3]-ddq_d[:,3],'g-')
    plt.subplot(615)
    plt.plot(i,ddq_d[:,4],'r--', i,ddq_r[:,4],'b:', i,ddq_r[:,4]-ddq_d[:,4],'g-')
    plt.subplot(616)
    plt.plot(i,ddq_d[:,5],'r--', i,ddq_r[:,5],'b:', i,ddq_r[:,5]-ddq_d[:,5],'g-')

    plt.figure(4)
    plt.title('trajectory')
    ax = plt.axes(projection='3d')
    p_d=np.zeros((n,3)) # 期望末端轨迹XYZ
    p_r=np.zeros((n,3)) # 实际末端轨迹XYZ
    for i in range(n):
        p_d[i,:]=ecr5.fkine(q_d[i,:]).t
        p_r[i,:]=ecr5.fkine(q_r[i,:]).t
    ax.plot3D(p_d[:,0],p_d[:,1],p_d[:,2], 'r--')
    ax.plot3D(p_r[:,0],p_r[:,1],p_r[:,2], 'b:')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')

    plt.show(block=True)


# MDH建模
# 输入DH参数，输出robot对象
def Model(a, alpha, d, theta):
    links=[]
    for i in range(6):
        l=rtb.RevoluteMDH(d=d[i], a=a[i], alpha=alpha[i], offset=theta[i])
        links.append(l)

    return rtb.DHRobot(links, name='ECR5')


# 关节轨迹规划
# 输入途径点序列和任务周期时间，输出插补后的轨迹(位移 速度 加速度)
def Planner(point, dt):
    point=point*pi/180
    traj = rtb.mstraj(viapoints=point, dt=dt, tacc=0.5, qdmax=[0.5]*6)  # 多段多轴轨迹插补
    n,m=traj.q.shape

    q_d=traj.q
    dq_d=np.zeros((n,m))
    ddq_d=np.zeros((n,m))
    for i in range(1,n):
        dq_d[i-1,:]=(q_d[i,:]-q_d[i-1,:])/dt    # 速度计算
    for i in range(1,n):
        ddq_d[i-1,:]=(dq_d[i,:]-dq_d[i-1,:])/dt # 加速度计算

    return q_d, dq_d, ddq_d

# 粒子群算法(优化自适应惯性权重)
# *******************************
# 输入：
# x_di dx_di ddx_di：i时刻期望位移 速度 加速度
# x_ri dx_ri：i时刻实际位移 速度
# f：i时刻各关节力矩偏差
# dt：采样间隔
# x_dii dx_dii ddx_dii：i+1时刻实际位移 速度 加速度
# *******************************
# 输出：
# gbest：导纳最优参数(m b k)
def Apso(x_di, dx_di, ddx_di, x_ri, dx_ri, f, dt, x_dii, dx_dii, ddx_dii):

    # 初始化
    step=15 # 进化次数
    n=20    # 种群规模
    m=3 # 粒子维度,即目标函数的自变量个数
    xlimit=np.array([[1,1,1],[20,100,500]])  # 目标函数自变量的取值范围
    vmax=np.array([1,2,3])    # 最大进化速度
    cost=np.array([np.inf]*n)   # 个体适应度代价
    pbest=np.zeros((n,m))    # 个体最值
    gbest=np.zeros((1,m))    # 群体最值
    particle=np.zeros((n,2,m)); # n个粒子，粒子每行分别表示位置、速度，每列对应一个自变量的迭代值
    for i in range(n):
        particle[i,0,:]=np.random.rand(1,m)*(xlimit[1,:]-xlimit[0,:]) + xlimit[0,:]
        particle[i,1,:]=np.random.rand(1,m)*vmax

    #step迭代
    for k in range(step):
        # 适应度评价
        for i in range(n):   # 循环n个粒子
            shi,vel,acc=Impedence(x_di, dx_di, ddx_di, x_ri, dx_ri, f, dt, particle[i,0,:])    # (i+1)t时刻的实际轨迹方案
            error=np.linalg.norm(shi-x_dii) + np.linalg.norm(vel-dx_dii) + np.linalg.norm(acc-ddx_dii) # 计算粒子当前代价
            if error < cost[i]:  # 粒子当前代价小于粒子最小代价
                cost[i]=error   # 更新粒子最小代价
                pbest[i,:]=particle[i,0,:]   # 更新粒子最优位置
        i=np.argmin(cost)   # 找到最小代价索引
        gbest=pbest[i,:]    # 将最小代价的个体最优位置赋值给群体最优位置
    
        # 个体进化
        c1=1.5  # 自身认知系数
        c2=1.5  # 群体认知系数
        wlimit=np.array([0.4, 0.9]) # 惯性权重的取值范围
        costavg=np.mean(cost)  # 代价平均值
        costmin=np.min(cost)    # 代价最小值
        
        for i in range(n):
            # 自适应惯性权重计算
            if cost[i]>costavg:
                w=wlimit[1]
            elif costavg-costmin==0:
                w=wlimit[0]+np.random.rand()*(wlimit[1]-wlimit[0])
            else:
                w=wlimit[0]+(wlimit[1]-wlimit[0])*(cost[i]-costmin)/(costavg-costmin)
            
            particle[i,1,:]=w*particle[i,1,:] + c1*np.random.rand(1,3)*pbest[i,:] + c2*np.random.rand(1,3)*gbest  # 个体速度进化
            particle[i,1,:]=np.minimum(particle[i,1,:], vmax)   # 最大速度进化约束
            particle[i,0,:]=particle[i,0,:] + particle[i,1,:]   # 个体位置进化
            particle[i,0,:]=np.maximum(particle[i,0,:], xlimit[0,:])    # 最小位置进化约束
            particle[i,0,:]=np.minimum(particle[i,0,:], xlimit[1,:])   # 最大速度进化约束
    
    return gbest


# 导纳控制
# *******************************
# 输入：
# x_d dx_d ddx_d：i时刻期望位移 速度 加速度
# x_r dx_r：i时刻实际位移 速度
# f：i时刻各关节力矩偏差
# dt：采样间隔
# param：控制参数(m b k)
# *******************************
# 输出：
# x_r dx_r ddx_r：i+1时刻实际位移 i+1时刻实际速度 i时刻实际加速度
def Impedence(x_d, dx_d, ddx_d, x_r, dx_r, f, dt, param):

    # 阻抗模型f = m*(ddx_d-ddx_r) + b*(dx_d-dx_r) + k*(x_d-x_r)
    # m,b,k分别为质量 阻尼 刚度
    # m=1
    # b=5
    # k=10
    m=param[0]
    b=param[1]
    k=param[2]

    ddx_r = ddx_d + (b*(dx_d-dx_r) + k*(x_d-x_r) - f)/m #n*t时刻
    x_r = x_r + dx_r * dt   #(n+1)*t时刻
    dx_r = dx_r + ddx_r * dt    #(n+1)*t时刻

    x_r=np.array([x_r])
    dx_r=np.array([dx_r])
    ddx_r=np.array([ddx_r])

    return x_r, dx_r, ddx_r


if __name__ == '__main__':    # pragma nocover
    Main()
