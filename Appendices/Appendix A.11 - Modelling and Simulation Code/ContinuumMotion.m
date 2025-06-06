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

function center = curve_center(x_fix, y_fix, fix_dir,r)
    if fix_dir == "y"
        center = [x_fix+r, y_fix];
    else
        center = [x_fix, y_fix-r];
    end
    return
end


function plot_robot(x, y, theta, fix_index)
    L = 600;
    plot(x(1), y(1), "x", 'Color', 'blue')
    plot(x(2), y(2), "x", 'Color', 'blue')
    dir = ["y", "x"];
    r = L/(theta);
    center = curve_center(x(fix_index), y(fix_index), dir(fix_index), r);
    if fix_index == 1
        th = linspace(theta, 0, 100);
        plot(-r*cos(th)+center(1),r*sin(th)+center(2), ":", "Color", "blue")
    else 
        th = linspace(pi/2, (pi/2-theta), 100);
        plot(-r*cos(th)+center(1),r*sin(th)+center(2), ":", "Color", "blue")
    end
end

function [x_new,y_new]=bend(theta, x_fix, y_fix, fix_dir)
    L = 600;
    r = L/theta;
    center = curve_center(x_fix, y_fix, fix_dir, r);
    if fix_dir == "y"
        x_new = center(1)-r*cos(theta);
        y_new = center(2)+r*sin(theta);
    else 
        x_new = center(1)-r*sin(theta);
        y_new = center(2)+r*cos(theta);
    end
    return
end

function [x_new, y_new] = unbend(theta_now, delta_theta, x_fix, y_fix, fix_dir)
    theta = theta_now - delta_theta;
    [x_new, y_new] = bend(theta, x_fix, y_fix, fix_dir);
    return
end

function plot_robot_no_bend(x, y)
    plot(x(1), y(1), "x", 'Color', 'blue')
    plot(x(2), y(2), "x", 'Color', 'blue')
    plot(x, y, ":", 'Color', 'blue')
end

figure
hold on
theta_global = 0;
subplot(4,4,1)
plot_gantry()
xjoints = [250, 250];
yjoints = [-250, 450];
plot_robot_no_bend(xjoints, yjoints)

subplot(4,4,2)
plot_gantry()
theta_global = theta_global + pi/6;
[xjoints(2), yjoints(2)] = bend(theta_global, xjoints(1), yjoints(1), "y");
plot_robot(xjoints,yjoints,theta_global, 1);

subplot(4,4,3)
plot_gantry()
yjoints = yjoints+100;
plot_robot(xjoints,yjoints,theta_global, 1);

subplot(4,4,4)
plot_gantry()
theta_global = theta_global + pi/6;
[xjoints(2), yjoints(2)] = bend(theta_global, xjoints(1), yjoints(1), "y");
plot_robot(xjoints,yjoints,theta_global, 1);

subplot(4,4,5)
plot_gantry()
yjoints=yjoints+50;
plot_robot(xjoints,yjoints,pi/3, 1);

subplot(4,4,6)
plot_gantry()
theta_global = 3*pi/8;
[xjoints(2), yjoints(2)] = bend(theta_global, xjoints(1), yjoints(1), "y");
plot_robot(xjoints,yjoints,theta_global, 1);

subplot(4,4,7)
plot_gantry()
yjoints=yjoints+50;
plot_robot(xjoints,yjoints,3*pi/8, 1);

subplot(4,4,8)
plot_gantry()
theta_global = pi/2;
[xjoints(2), yjoints(2)] = bend(theta_global, xjoints(1), yjoints(1), "y");
plot_robot(xjoints,yjoints,theta_global, 1);

subplot(4,4,9)
plot_gantry()
[xjoints(1), yjoints(1)] = unbend(theta_global, pi/8, xjoints(2), yjoints(2), "x");
theta_global = theta_global - pi/8;
plot_robot(xjoints,yjoints,theta_global, 2);

subplot(4,4,10)
plot_gantry()
xjoints = xjoints + 100;
plot_robot(xjoints,yjoints,theta_global, 2);

subplot(4,4,11)
plot_gantry()
[xjoints(1), yjoints(1)] = unbend(theta_global, pi/8, xjoints(2), yjoints(2), "x");
theta_global = theta_global - pi/8;
plot_robot(xjoints,yjoints,theta_global, 2);

subplot(4,4,12)
plot_gantry()
xjoints = xjoints + 100;
plot_robot(xjoints,yjoints,theta_global, 2);

subplot(4,4,13)
plot_gantry()
yjoints(1) = yjoints(2);
xjoints(1) = xjoints(2)-600;
theta_global = theta_global - pi/4;
plot_robot_no_bend(xjoints,yjoints);

subplot(4,4,14)
plot_gantry()
xjoints = xjoints + 100;
plot_robot_no_bend(xjoints,yjoints);

subplot(4,4,15)
plot_gantry()
xjoints = xjoints + 100;
plot_robot_no_bend(xjoints,yjoints);

subplot(4,4,16)
plot_gantry()
xjoints = xjoints + 100;
plot_robot_no_bend(xjoints,yjoints);