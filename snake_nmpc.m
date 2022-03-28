%蛇型ロボットの各パラメータ
num_joint = 30;
length_quarter = 5.0;
length_section = 17.0325;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

%NMPCの各パラメータ
goal_error = 1.0e-2;
dt = 0.01;
iteration_time = 30;
iteration_num = iteration_time/dt;

%目標状態[x;y;theta]
goal_pos = [40; 40; 0];

%初期状態[x;y;theta]
init_X = [0; 0; 0];

nmpc = NMPC_two_wheel(init_X, goal_pos);

%CGMRESの計算
for i = 1:iteration_num-1
    time = i*dt;
    disp(time)
    u = nmpc.CGMRES(time, goal_pos);
    nmpc.updateState(u, dt);
end


%曲率を計算
%now_pos = [nmpc.save_x(1, 1:2); nmpc.save_x(:,1:2)];
%[~, curvature_nmpc, ~] = curvature(nmpc.save_x(:,1:2));
%curvature_nmpc = 1/curvature_nmpc;

%体形曲線の生成
ini_theta=atan2(nmpc.save_x(2, 2)-nmpc.save_x(1, 2), nmpc.save_x(2, 1)-nmpc.save_x(1, 1));
snake = SnakeRobot(num_joint, length_quarter, ini_theta+alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);
curvature_nmpc = 0;
for i = 4:size(nmpc.save_x, 1)
    tempx0=nmpc.save_x(i-3, 1);
    tempx1=nmpc.save_x(i-2, 1);
    tempx2=nmpc.save_x(i-1, 1);
    tempy0=nmpc.save_x(i-3, 2);
    tempy1=nmpc.save_x(i-2, 2);
    tempy2=nmpc.save_x(i-1, 2);
    curvature_yaw = -calCurvature(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2);
    curvature_nmpc = [curvature_nmpc; curvature_yaw];
    diff = nmpc.save_x(i-2, 1:2) - nmpc.save_x(i-3, 1:2);
    vel = norm(diff);
    snake.path2Param(vel, curvature_yaw);
    snake.changeAlphaYaw(alpha_yaw);
    snake.updateModel();
end

snake_path = snake.plotData(); 
disp("path")
disp(nmpc.save_x(:,1:2))
disp("path_end")
%{
temp1 = 1:size(curvature_nmpc, 1);
temp1 = temp1';
temp2 = [temp1 curvature_nmpc];
disp(temp2)
%}

%表示
tiledlayout(4, 1)

nexttile
plot(nmpc.save_x(:, 1), nmpc.save_x(:, 2))
hold on
plot(snake_path(:, 1), snake_path(:, 2));
hold off
daspect([1 1 1])
title("composite")

nexttile
plot(nmpc.save_x(:, 1), nmpc.save_x(:, 2))
daspect([1 1 1])
title("path")

nexttile
plot(snake_path(:, 1), snake_path(:, 2));
daspect([1 1 1])
title("snake")

nexttile
plot(curvature_nmpc)
daspect([1 1 1])
title("curvature")