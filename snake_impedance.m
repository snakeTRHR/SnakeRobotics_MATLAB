%蛇型ロボットの各パラメータ
num_joint = 12;
%length_one_jointは整数じゃないとバグる
length_one_joint = 2.0;
length_quarter = 5.0;
alpha_yaw = pi/3;
alpha_pitch = 0.0;
dim = 2;

snake=SnakeRobot(num_joint, length_one_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

length_total = 48;

%とりあえずサーペノイド曲線の生成
for i = 1:length_total
    snake.updateModel();
end

disp(snake.joint_radlog_y)
%理想曲線を離散化
snake.calDiscretization();

%グラフデータを生成


%各フレームの画像データを生成
fig = figure;
lines = [animatedline('Color', 'red'); animatedline('Color', 'blue'); animatedline('Color', 'green')];
frames(100) = struct('cdata', [], 'colormap', []);
t_start = tic;
t_end = 0;
for i = 1:1000
    %実際曲線を離散化
    snake.calDiscretizationNoise();

    plot(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2))
    hold on
    plot(snake.discretization_pathlog(:,1), snake.discretization_pathlog(:,2))
    plot(snake.discretization_pathlog(:,1), snake.discretization_pathlog(:,2), 'o')
    plot(snake.discretization_noise_pathlog(:,1), snake.discretization_noise_pathlog(:,2))
    plot(snake.discretization_noise_pathlog(:,1), snake.discretization_noise_pathlog(:,2), 'o')
    for joint_num = 1:size(snake.discretization_pathlog, 1)
        plot([snake.discretization_pathlog(joint_num, 1), snake.discretization_noise_pathlog(joint_num, 1)], [snake.discretization_pathlog(joint_num, 2), snake.discretization_noise_pathlog(joint_num, 2)], 'g');
    end

    %インピーダンス制御
    t_end = toc(t_start);
    snake.calImpedance(t_end);
    t_start = tic;
    for k = 1:num_joint
        disp([snake.impedance_x(k, 1), snake.impedance_dx(k, 1), snake.impedance_ddx(k, 1), snake.impedance_f_u(k, 1), snake.discretization_anglelog(k, 1)*180/pi, snake.impedance_tau(k, 1)])
    end
    axis equal
    grid on
    hold off

    drawnow;
    frames(i) = getframe(gcf);

    snake.logClear();
end

%mp4で出力する
video = VideoWriter('impedance.mp4', 'MPEG-4');
open(video);
writeVideo(video, frames);
close(video);