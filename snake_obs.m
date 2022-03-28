%蛇型ロボットの各パラメータ
num_joint = 30;
length_quarter = 5.0;
length_section = 17.0325;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

snake = SnakeRobot(num_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

%フィールドデータの読み込み
load fieldDataZ.mat

%目標値と現在地の許容誤差
error = 1;

while(goal-now<error)
    snake.
    snake.updateModel();
end

snake.plotSnake();
disp(snake.calCycleLength(alpha_yaw, alpha_pitch ,length_quarter));
