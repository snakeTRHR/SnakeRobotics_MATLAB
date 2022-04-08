%蛇型ロボットの各パラメータ
num_joint = 30;
length_quarter = 5.0;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

snake = SnakeRobot(num_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

%障害物の設定
obs2d=[ 4, 5, 0;
        8, 20, 0;
       10, 45, 0;
        8, 70, 0;
        8, 90, 0];

%目標値と現在地の許容誤差
error = 1;

%とりあえずサーペノイド曲線を生成
for i=1:150
    snake.updateModel();
end

plot(snake.snake_pathlog(:,1), snake.snake_pathlog(:,2), 'r')
hold on
obsData=[0, 0];
for i=1:size(obs2d, 1)
    r=obs2d(i,1);
    x0=obs2d(i,2);
    y0=obs2d(i,3);
    for x=x0-r:0.1:x0+r
        y_max=y0+sqrt(r^2-(x-x0)^2);
        y_min=y0-sqrt(r^2-(x-x0)^2);
        plot(x, y_min)
        plot(x, y_max)
        obsData=[obsData; 
                 x, y_min; 
                 x, y_max];
    end
end
obsData=obsData(1:end,:);
plot(obsData(:,1), obsData(:,2))
grid on
axis equal
hold off

%負荷グラフの作成
% nearest_obs=findNearestObs();

%dtheta/dtを決定

