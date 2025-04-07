%lqr线性化函数(返回参数：lqr参数，对应车辆速度，参数组数）
function [K_,V_,C_]=bike2dof_lqr_model1(Ts,top,bottom,step,lqr_Q,lqr_R)
count=0;
for n=bottom:step:top
    count=count+1;
end

%建立单元数组，开辟空间
K_=cell(1,count);
V_=zeros(1,count);
C_=count;

xNum = size(lqr_Q,1);%获取行数，也就是状态变量个数
uNum = size(lqr_R,2);%获取列数，也就是输入变量个数

g = 9.8;
h = 0.4631022;
w = 1.02065;
b = 0.4651025;
% 重力加速度
% 自行车质心的高度
% 自行车的行驶速度
% 前后轮与地面接触点之间的距离
% 后轮的着地点与重心在地面上的投影间的距离



m=1;
for v1_=bottom:step:top
    A_21 = g/h;
    A_23 = v1_^2/(w*h);
    B_21 = (b*v1_)/(w*h);
    lqr_A = [0 1 0; A_21 0 A_23; 0 0 0];
    lqr_B = [0; B_21; 1];
    lqr_C = [1 0 0; 0 0 1];
    lqr_D = 0;
    sys_c = ss(lqr_A,lqr_B,lqr_C,lqr_D);
    sys_d = c2d(sys_c, Ts);
    %判断可控性
    if (rank(ctrb(sys_d.A,sys_d.B))==xNum)    
        temp=dlqr(sys_d.A,sys_d.B,lqr_Q,lqr_R);
        cell_temp = cell(1,1);
        for i = 1:uNum
            for j = 1:xNum
                cell_temp{1}(i, j) = temp(i, j);
            end
        end
        K_(1,m)=cell_temp;
        V_(1,m)=v1_;
    else
        K_(1,m)=0;
        V_(1,m)=v1_;
        disp('Uncontrollable!');
    end
    m = m+1;
end


end