%蛇型ロボットの各パラメータ
num_joint = 30;
length_quarter = 5.0;
alpha_yaw = pi/4;
alpha_pitch = 0.0;
dim = 2;

snake = SnakeRobot(num_joint, length_quarter, alpha_yaw, alpha_pitch, dim);
snake.changeVel(1);

%障害物の設定
%obs2d=[r, x, y, flag(1:+, -1:-, 0:未定)]
obs2d=[10, 45, 0, 0;
        8, 90, 0, 0];

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
% for i=1:size(snake.snake_pathlog, 1)
%     x_path=round(snake.snake_pathlog(i, 1))+offset_x_obs;
%     y_path=round(snake.snake_pathlog(i, 2))+offset_y_obs;
%     %disp([x_path-offset_x_obs, y_path-offset_y_obs])
%     if obs_map(x_path, y_path) == 1
%         nearest_obs=findNearestObs(obs2d, snake.snake_pathlog(i, 1), snake.snake_pathlog(i, 2));
%         r=nearest_obs(1, 1);
%         x0=nearest_obs(1, 2);
%         y0=nearest_obs(1, 3);
%         %disp([x0 y0, x_path, y_path])
%         temp_y=y0+sqrt(r^2-(snake.snake_pathlog(i, 1)-x0)^2);
%         disp(nearest_obs)
%         load=temp_y-snake.snake_pathlog(i, 2);
%     else
%         load=0;
%     end
%     trajectory_load=[trajectory_load; 
%                      snake.snake_pathlog(i, 1), load];
% end
for i=1:size(snake.snake_pathlog, 1)
%     %符号の決定
%     for k=1:size(obs2d, 1)
%         if obs2d(k, 4)==0
%             if snake.snake_pathlog(i, 2)>0
%                 obs2d(k, 4)=1;
%             else
%                 disp([snake.snake_pathlog(i, 1), snake.snake_pathlog(i, 2)])
%                 obs2d(k, 4)=-1;
%             end
%         end
%     end


    %nearest=[flag(障害物内に入っているかの判定), obs]
    serpen_x_=snake.snake_pathlog(i, 1);
    serpen_y_=snake.snake_pathlog(i, 2);
    dist=sqrt((serpen_x_-obs2d(1, 2))^2+(serpen_y_-obs2d(1, 3))^2);
    if dist<obs2d(1, 1)
        %符号の決定
        if obs2d(1, 4)==0
            if snake.snake_pathlog(i, 2)>0
                obs2d(1, 4)=1;
            else
                obs2d(1, 4)=-1;
            end
        end
        nearest=[1, obs2d(1,:)];
    else
        nearest=[0, obs2d(1,:)];
    end
    min_dist=sqrt((serpen_x_-obs2d(1, 2))^2+(serpen_y_-obs2d(1, 3))^2);
    for k=2:size(obs2d, 1)
        dist=sqrt((serpen_x_-obs2d(k, 2))^2+(serpen_y_-obs2d(k, 3))^2);
        if dist<min_dist
            if dist<obs2d(k, 1)
                %符号の決定
                if obs2d(k, 4)==0
                    if snake.snake_pathlog(i, 2)>0
                        obs2d(k, 4)=1;
                    else
                        obs2d(k, 4)=-1;
                    end
                end
                nearest=[1, obs2d(k, :)];
            else
                nearest=[0, obs2d(k, :)];
            end
            min_dist=dist;
        end
    end

    nearest_obs=nearest;
    %nearest_obs=findNearestObs(obs2d, snake.snake_pathlog(i, 1), snake.snake_pathlog(i, 2));
    if  nearest_obs(1, 1) == 1
        r=nearest_obs(1, 2);
        x0=nearest_obs(1, 3);
        y0=nearest_obs(1, 4);
        if nearest_obs(1, 5)==1
            temp_y=y0+sqrt(r^2-(snake.snake_pathlog(i, 1)-x0)^2);
        else
            temp_y=y0-sqrt(r^2-(snake.snake_pathlog(i, 1)-x0)^2);
        end
        load=temp_y-snake.snake_pathlog(i, 2);
    else
        load=0;
    end
    trajectory_load=[trajectory_load; 
                     snake.snake_pathlog(i, 1), load];
end
trajectory_load=trajectory_load(1:end, :);

%グラフの表示
tiledlayout(3, 1)

nexttile
plot(obsData(:, 1), obsData(:, 2),'b')
hold on
plot(snake.snake_pathlog(:, 1), snake.snake_pathlog(:, 2), 'r')
grid on
axis equal
title('not activate avoidance')
hold off

nexttile
plot(obsData(:, 1), obsData(:, 2),'b')
hold on
plot(snake.snake_pathlog(:, 1), snake.snake_pathlog(:, 2), 'r')
plot(trajectory_load(:, 1), trajectory_load(:, 2), 'g')
axis equal
grid on
hold off
title('load compound')

nexttile
plot(trajectory_load(:, 1), trajectory_load(:, 2), 'g')
axis equal
grid on
title('load')

%dtheta/dtを決定

% function ans_obs=findNearestObs(obs_, serpen_x_, serpen_y_)
%     %nearest=[flag(障害物内に入っているかの判定), obs]
%     dist=sqrt((serpen_x_-obs_(1, 2))^2+(serpen_y_-obs_(1, 3))^2);
%     if dist<obs_(1, 1)
%         nearest=[1, obs_(1,:)];
%     else
%         nearest=[0, obs_(1,:)];
%     end
%     min_dist=sqrt((serpen_x_-obs_(1, 2))^2+(serpen_y_-obs_(1, 3))^2);
%     for i=2:size(obs_, 1)
%         dist=sqrt((serpen_x_-obs_(i, 2))^2+(serpen_y_-obs_(i, 3))^2);
%         if dist<min_dist
%             if dist<obs_(i, 1)
%                 nearest=[1, obs_(i, :)];
%             else
%                 nearest=[0, obs_(i, :)];
%             end
%             min_dist=dist;
%         end
%     end
% 
%     ans_obs=nearest;
% end
