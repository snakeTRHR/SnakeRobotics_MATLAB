%obs2d=[r, x, y]
obs2d=[ 4, 5, 0;
        8, 20, 0;
       10, 45, 0;
        8, 70, 0;
        8, 90, 0];

map_obs2d=zeros(400, 400);
offset_x2d=200;
offset_y2d=200;

for i=1:size(obs, 1)
    r=obs(i,1);
    x0=obs(i,2);
    y0=obs(i,3);
    for x=x0-r:0.1:x0+r
        y_max=y0+sqrt(r^2-(x-x0)^2)+offset_y;
        y_min=y0-sqrt(r^2-(x-x0)^2)+offset_y;
        y_max_=round(y_max);
        y_min_=round(y_min);
        x_=round(x+offset_x);
        disp(x_)
        map_obs(x_,y_min_:y_max_)=100;
    end
end