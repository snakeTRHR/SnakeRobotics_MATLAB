r=10;
x0=15;
y0=15;
map=zeros(100,100);

%y=y0+sqrt(r^2+(x-x0)^2)
X=0;
Y=0;
for x=x0-r:0.1:x0+r
    y_max=y0+sqrt(r^2-(x-x0)^2);
    y_min=y0-sqrt(r^2-(x-x0)^2);
    y_max_=round(y_max);
    y_min_=round(y_min);
    x_=round(x);
    map(x_,y_min_:y_max_)=100;
    X=[X;x_];
    Y=[Y;y_max_];
end
%plot graph
plotData=zeros(1, 3);
for x=1:100
    for y=1:100
        plotData=[plotData;
                  x,y,map(x,y)];
    end
end
plotDdata=plotData(2:end,:);
surfc(1:100, 1:100, map)
