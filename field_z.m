x = [-200; -200;    0; 0;   5; 20; 45;  70; 90; 200;   0; 200];
y = [-200;    0; -200; 0;   0;  0;  0;   0;  0;   0; 200; 200];
z = [   0;    0;    0; 0;  50; 50; 50;  50; 50;   0;   0;   0];

[xData, yData, zData] = prepareSurfaceData( x, y, z );

ft = 'thinplateinterp';

[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

resultMatZ=zeros(400,400);
for x = 1:400
    for y = 1:400
        disp(x)
        resultMatZ(x,y) = fitresult((x-200)/10, (y-200)/10);
    end
end

plot(fitresult)
xlabel("x")
ylabel("y")
zlabel("z")
disp(resultMatZ)
save('fieldDataZ', 'resultMatZ')