%obs=[r, x, y, z]
obs=[ 4,  5, 0, 0;
      8, 20, 0, 0;
     10, 45, 0, 0;
      8, 70, 0, 0;
      8, 90, 0, 0];

% obs=[ 5, 10, 0, 0;
%      20, 60, 0, 0];

%[-200,-200],[200,200]のフィールド
map_obs=zeros(400,400);
offset_x=200;
offset_y=200;

%y=y0+sqrt(r^2+(x-x0)^2)
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

surfc(1:400, 1:400, map_obs)

save('fieldDataZObs','map_obs')
