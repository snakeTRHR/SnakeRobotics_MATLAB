%蛇型ロボットの各パラメータ
num_joint = 30;
%length_one_jointは整数じゃないとバグる
length_one_joint = 2.0;
length_quarter = 5.0;
alpha_yaw = pi/3;
alpha_pitch = pi/6;
alpha_yaw_ini = pi/3;
alpha_pitch_ini = pi/6;
dim = 3;

pub_effort = rospublisher('effort', 'std_msgs/Float32MultiArray');
pub_head_pos = rospublisher('head_pos', 'geometry_msgs/Pose2D');
publish_effort = rosmessage(pub_effort);
publish_head_pos = rosmessage(pub_head_pos);

snake=SnakeRobot(num_joint, length_one_joint, length_quarter, alpha_yaw, alpha_pitch, alpha_yaw_ini, alpha_pitch_ini, dim);
snake.changeVel(1);
r = rosrate(30);
reset(r)

for i = 1:1000
    snake.updateModel();
    snake.calDiscretization();
    plot3(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2), snake.snake_pathlog(:,3))
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    drawnow
    %ros
    for p =1:num_joint
        publish_effort.Data(p, 1)=snake.joint_radlog(p, 1);
    end
    [publish_head_pos.X, publish_head_pos.Y] = snake.getHeadPos();
    send(pub_effort, publish_effort);
    send(pub_head_pos, publish_head_pos);
    waitfor(r);
end
