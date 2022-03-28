num_joint = 30;
length_quarter = 5.0;
length_section = 17.0325;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

snake = SnakeRobot(num_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

for i = 1:20
    snake.updateModel();
end

snake.plotSnake();
