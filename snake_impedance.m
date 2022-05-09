%蛇型ロボットの各パラメータ
num_joint=30;
length_one_joint=2.5;
length_quarter=5.0;
alpha_yaw=pi/4;
alpha_pitch=0.0;
dim=2;

snake = SnakeRobot(num_joint, length_one_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

length_total=20;

%とりあえずサーペノイド曲線の生成
for i=1:length_total
    snake.updateModel();
end
%snake.plotSnake();
%離散化
joint_data=zeros(round(length_total/length_one_joint), 1);
for i=1:round(80/length_one_joint)
    joint_data(i,1)=2*length_one_joint*snake.snakeCurvatureYaw(i*length_one_joint);
end

disp(joint_data*180/pi);

%離散化表示
pos_discretization=zeros(round(length_total/length_one_joint), 2);
for i=1:round(length_total/length_one_joint)
    if(i==1)
        pos_discretization(i,1)=length_one_joint*cos(joint_data(i, 1));
        pos_discretization(i,2)=length_one_joint*sin(joint_data(i, 1));
    else
        pos_discretization(i,1)=pos_discretization(i-1,1)+length_one_joint*cos(joint_data(i, 1));
        pos_discretization(i,2)=pos_discretization(i-1,2)+length_one_joint*sin(joint_data(i, 1));
    end
end
% plot(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2))
% hold on
% plot(pos_discretization(:,1),pos_discretization(:,2))
% axis equal
% grid on
% hold off
disp(snake.joint_radlog_y)
snake.calDiscretization();
plot(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2))
hold on
plot(snake.discretization_pathlog(:,1), snake.discretization_pathlog(:,2))
plot(snake.discretization_pathlog(:,1), snake.discretization_pathlog(:,2), 'o')
axis equal
grid on
hold off
disp(snake.discretization_pathlog)
disp(size(snake.joint_radlog_y, 1))
disp(snake.joint_radlog_y*180/pi)
%座標変換