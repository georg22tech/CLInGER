%%% PLOT DATA %%%
%read in csv file for values
T = readtable('imu_data.csv');
%extract x and y values respectivley
x = (table2array(T(:,"qx")));
y = (table2array(T(:,"qy")));
%normalise to (0,0) origin
x = x - x(1);
y = y - y(1);
%rotate 180 degrees round origin
xr = x*cos(3*pi/2) - y*sin(3*pi/2);
yr = x*sin(3*pi/2) + y*cos(3*pi/2);
x = xr;
y = yr;
figure
hold on
plot(x,y)
%predicted x and y values
th = linspace(0, pi/2, length(x));
R = 0.43;
xdt = R*cos(th);
ydt = R*sin(th);
%rotate
xd = xdt*cos(pi/2) - ydt*sin(pi/2);
yd = xdt*sin(pi/2) + ydt*cos(pi/2); 
%shift to (0,0) origin
xd = xd - xd(1);
yd = yd - yd(1);
plot(xd, yd,'Color','red','LineStyle','--');
plot(0,0,'o','Color', 'green')
%%% CALCULATE PERCETAGE DEVIATION %%%
%find deviations
e_x = xd - x';
e_y = yd - y';
%find a scalar total error
e_sum = sqrt(e_x*e_x'+e_y*e_y');
%find vector of error values and get as a percentage
e = sqrt(e_x.^2+e_y.^2);
percent_e_x = zeros(1,length(e_x));
percent_e_y = zeros(1,length(e_y));
for i = 1:length(x)
    percent_e_x(i) = (e_x(i)/x(i))*100;
    percent_e_y(i) = (e_y(i)/y(i))*100;
end
percent_e_x(1) = 0;
percent_e_y(1)  = 0;
percent_e = sqrt(percent_e_x.^2+percent_e_y.^2);
figure
hold on
step = linspace (0, length(e_x), length(e_x));
plot(step, percent_e)
%%% ESTIMATE mm ERROR %%%
scalar = 250/abs((y(1)-y(length(y))));
e_mm = e*scalar;
figure
hold on
t = linspace (0, 31, length(e_x));
plot (t, e_mm);
xlabel('time (s)')
ylabel('estimated error (mm)')