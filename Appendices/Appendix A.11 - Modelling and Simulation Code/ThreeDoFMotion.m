function plot_gantry ()
    hold on
    %cross section plotting
    line([0 0], [-200 600], 'Color','black', 'LineWidth', 3);
    line([0 100], [100 100], 'Color','black', 'LineWidth', 3);
    line([0 800],[600 600], 'Color','black', 'LineWidth', 3);
    line([400 800],[100 100], 'Color','black', 'LineWidth', 3);
    line([500 500], [-200 200], 'Color','black', 'LineWidth', 3);
    line([500 500], [500 600], 'Color','black', 'LineWidth', 3);
    %limits plotting
    line([100 100], [200 500],'Color','red','LineStyle',':');
    line([100 100], [-200 0],'Color','red','LineStyle',':');
    line([400 400], [-200 0],'Color','red','LineStyle',':');
    line([100 400], [500 500],'Color','red','LineStyle',':');
    line([600 800], [500 500],'Color','red','LineStyle',':');
    line([600 800], [200 200],'Color','red','LineStyle',':');
    th = linspace( pi/2, -pi/2, 100);
    R = 100;
    plot(R*cos(th) + 100, R*sin(th) + 100,'Color','red','LineStyle',':');
    plot(-R*cos(th) + 400, R*sin(th) + 100,'Color','red','LineStyle',':');
    th = linspace( 0, pi, 100);
    plot(R*cos(th) + 500, R*sin(th) + 200,'Color','red','LineStyle',':');
    plot(R*cos(th) + 500, -R*sin(th) + 500,'Color','red','LineStyle',':');
    axis square
    % %labels
    xlim([0,800])
    ylim([-200,600])
    xlabel("Distance/mm")
    ylabel("Distance/mm")
end

function plot_robot(x, y)
    plot(x(1), y(1), "x", 'Color', 'blue')
    plot(x(2), y(2), "o", 'Color', 'blue')
    plot(x(3), y(3), "x", 'Color', 'blue')
    plot(x, y, ":", 'Color', 'blue')
end

function [x_new,y_new]=rotate(x,y,x_c, y_c, theta)
    R = [cos(theta) -sin(theta) 0;
        sin(theta) cos(theta) 0;
        0 0 1;];
    p_1 = [1 0 x_c; 0 1 y_c; 0 0 1];
    p_2 = [1 0 -x_c; 0 1 -y_c; 0 0 1];
    H = p_1*R*p_2;
    out = H*[x;y;1];
    x_new = out(1);
    y_new = out(2);
    return;
end

rotate(1, 1, 0, 0, pi/2)
figure
hold on
subplot(5,5,1)
plot_gantry()
xjoints = [250, 250, 250];
yjoints = [-150, 132, 414];
plot_robot(xjoints, yjoints)
subplot(5,5,2)
plot_gantry()
[xjoints(3), yjoints(3)] = rotate(xjoints(3), yjoints(3), xjoints(2), yjoints(2), -pi/4);
plot_robot(xjoints, yjoints)
subplot(5,5,3)
plot_gantry()
yjoints(2) = yjoints(2)+50;
yjoints(3) = yjoints(3)+50;
plot_robot(xjoints, yjoints)
subplot(5,5,4)
plot_gantry()
[xjoints(3), yjoints(3)] = rotate(xjoints(3), yjoints(3), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,5)
plot_gantry()
yjoints(2) = yjoints(2)+50;
yjoints(3) = yjoints(3)+50;
plot_robot(xjoints, yjoints)
subplot(5,5,6)
plot_gantry()
[xjoints(3), yjoints(3)] = rotate(xjoints(3), yjoints(3), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,7)
plot_gantry()
yjoints = yjoints + 50;
plot_robot(xjoints, yjoints)
subplot(5,5,8)
plot_gantry()
[xjoints(3), yjoints(3)] = rotate(xjoints(3), yjoints(3), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,9)
plot_gantry()
yjoints = yjoints + 50;
plot_robot(xjoints, yjoints)
subplot(5,5,10)
plot_gantry()
[xjoints(3), yjoints(3)] = rotate(xjoints(3), yjoints(3), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,11)
plot_gantry()
xjoints(3) = xjoints(3) + 150;
plot_robot(xjoints, yjoints)
subplot(5,5,12)
plot_gantry()
yjoints(2) = 350;
yjoints(3) = 350;
yjoints(1) = 350-282;
plot_robot(xjoints, yjoints)
subplot(5,5,13)
plot_gantry()
xjoints = xjoints+45;
xjoints(3) = xjoints(3) -45;
plot_robot(xjoints, yjoints)
subplot(5,5,14)
plot_gantry()
[xjoints(1), yjoints(1)] = rotate(xjoints(1), yjoints(1), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,15)
plot_gantry()
xjoints = xjoints+45;
xjoints(3) = xjoints(3) -45;
plot_robot(xjoints, yjoints)
subplot(5,5,16)
plot_gantry()
[xjoints(1), yjoints(1)] = rotate(xjoints(1), yjoints(1), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,17)
plot_gantry()
xjoints = xjoints+45;
xjoints(3) = xjoints(3) -45;
plot_robot(xjoints, yjoints)
subplot(5,5,18)
plot_gantry()
[xjoints(1), yjoints(1)] = rotate(xjoints(1), yjoints(1), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,19)
plot_gantry()
xjoints = xjoints+45;
xjoints(3) = xjoints(3) -15;
plot_robot(xjoints, yjoints)
subplot(5,5,20)
plot_gantry()
[xjoints(1), yjoints(1)] = rotate(xjoints(1), yjoints(1), xjoints(2), yjoints(2), -pi/16);
plot_robot(xjoints, yjoints)
subplot(5,5,21)
plot_gantry()
xjoints = xjoints+45;
plot_robot(xjoints, yjoints)
subplot(5,5,22)
plot_gantry()
[xjoints(1), yjoints(1)] = rotate(xjoints(1), yjoints(1), xjoints(2), yjoints(2), -pi/4);
plot_robot(xjoints, yjoints)
subplot(5,5,23)
plot_gantry()
xjoints = xjoints + 100;
plot_robot(xjoints, yjoints)
subplot(5,5,24)
plot_gantry()
xjoints = xjoints + 100;
plot_robot(xjoints, yjoints)
subplot(5,5,25)
plot_gantry()
xjoints = xjoints + 100;
plot_robot(xjoints, yjoints)