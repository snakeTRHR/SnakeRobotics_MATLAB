%蛇型ロボットの各パラメータ
num_joint = 30;
length_quarter = 5.0;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

snake = SnakeRobot(num_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

%障害物の設定
obs2d=[ 4,  5, 0;
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

%障害物の設定
obsData=[0, 0];
obs_map=zeros(400, 400);
offset_x_obs=200;
offset_y_obs=200;
for i=1:size(obs2d, 1)
    r=obs2d(i,1);
    x0=obs2d(i,2);
    y0=obs2d(i,3);
    for x=x0-r:0.1:x0+r
        y_max=y0+sqrt(r^2-(x-x0)^2);
        y_min=y0-sqrt(r^2-(x-x0)^2);
        x_=round(x);
        y_min_=round(y_min+offset_y_obs);
        y_max_=round(y_max+offset_y_obs);
        x_=round(x+offset_x_obs);
        obs_map(x_, y_min_:y_max_)=1;
        obsData=[obsData; 
                 x, y_min; 
                 x, y_max];
    end
end
obsData=obsData(1:end, :);

%負荷グラフの作成
%trajectory_load=[x, load]
trajectory_load=[0, 0];
for i=size(snake.snake_pathlog, 1)
    if obs_map(round(snake.snake_pathlog(i, 1))+offset_x_obs, round(snake.snake_pathlog(i, 2))+offset_y_obs) == 1
        nearest_obs=findNearestObs(obs2d, snake.snake_pathlog(i, 1), snake.snake_pathlog(i, 2));
        r=nearest_obs(1, 1);
        x0=nearest_obs(1, 2);
        y0=nearest_obs(1, 3);
        temp_y=y0+sqrt(r^2-(i-x0)^2);
        load=temp_y-snake.snake_pathlog(i, 2);
    else
        load=0;
    end
    trajectory_load=[trajectory_load; 
                     snake.snake_pathlog(i, 1), load];
end
trajectory_load=trajectory_load(1:end, :);
%グラフの表示
tiledlayout(2, 1)

nexttile
plot(snake.snake_pathlog(:, 1), snake.snake_pathlog(:, 2), 'r')
hold on
plot(obsData(:, 1), obsData(:, 2),'b')
grid on
axis equal
title('not activate avoidance')
hold off

nexttile
plot(trajectory_load(:, 1), trajectory_load(:, 2), 'g')
axis equal
grid on
title('activate avoidance')

%dtheta/dtを決定

function ans_obs=findNearestObs(obs_, serpen_x_, serpen_y_)
    nearest=[0, 0, 0];
    min_dist=0;
    for i=1:size(obs_, 1)
        dist=sqrt((serpen_x_-obs_(i, 2))^2+(serpen_y_-obs_(i, 3))^2);
        if dist<min_dist
            nearest=obs_(i, :);
            min_dist=dist;
        end
    end
    ans_obs=nearest;
end
