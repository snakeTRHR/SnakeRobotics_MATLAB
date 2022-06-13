%蛇型ロボットの各パラメータ
num_joint = 30;
%length_one_jointは整数じゃないとバグる
length_one_joint = 2.0;
length_quarter = 5.0;
alpha_yaw = pi/3;
alpha_pitch = 0.0;
dim = 2;

matlab_pub = rospublisher('effort', 'std_msgs/Float32MultiArray');
publish_command = rosmessage(matlab_pub);

snake=SnakeRobot(num_joint, length_one_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);
r = rosrate(5);
reset(r)
for i = 1:1000
    snake.updateModel();
    snake.calDiscretization();
    plot(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2))
    axis equal
    drawnow
    %ros
    for p =1:num_joint
        publish_command.Data(p, 1)=snake.joint_radlog(p, 1);
    end
    send(matlab_pub, publish_command);
    waitfor(r);
end
