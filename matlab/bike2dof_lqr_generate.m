%% 拟合lqr
clc;
%设定轮杆长度
v_top=5.;
v_bottom=0.5;
v_step=0.01;
Ts = 0.0025;

lqr_Q = [10000 0 0; ...
         0 0 0; ...
         0 0 200];
lqr_R = 1;

robot_type = 1;

%取出拟合lqr参数
[K_s,V_s,C_s] = bike2dof_lqr_model1 (Ts,v_top,v_bottom,v_step,lqr_Q,lqr_R,robot_type);

xNum = size(lqr_Q,1);%获取行数，也就是状态变量个数
uNum = size(lqr_R,2);%获取列数，也就是输入变量个数
Kp = cell(uNum,xNum);%构建参数元组

for j=1:1:uNum
    for k=1:1:xNum
        for i=C_s:-1:1
            Kp{j,k}(i)=K_s{1,i}(j,k);
        end
        figure;
        scatter(V_s, Kp{j,k}, 40, 'b', 'filled');  % 'filled'表示填充点
        title('参数散点图');
        xlabel('v');
        ylabel('k');
        grid on;
        hold on;
        coefficients = polyfit(V_s, Kp{j,k}, 7);
        fittedY = polyval(coefficients, V_s);
        plot(V_s, fittedY, 'r-', 'LineWidth', 2);
%         disp(coefficients)
%         % 定义对数函数模型：y = a + b*ln(x)
%         modelFun = @(b, x) b(1) + b(2)*log(x);  
%         initialParams = [1, 1];  % 初始参数猜测
%         
%         % 使用fitnlm函数拟合
%         nonlinModel = fitnlm(V_s, Kp{j,k}, modelFun, initialParams);
%         disp(nonlinModel.Coefficients);  % 显示参数估计值和统计量
%         
%         % 绘制拟合结果
%         plot(nonlinModel);
        bike2dof_lqr_fit_feedback(V_s,Kp{j,k},"/*参数*/",strcat("this->bike_lqr_params.lqrs[",num2str(k-1),"]"));
    end
end