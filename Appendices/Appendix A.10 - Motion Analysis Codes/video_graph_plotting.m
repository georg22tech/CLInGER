%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MOTION PLOT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read in csv file for values
T = readtable('final_continuum_1.csv');
%extract x and y values respectivley
x = medfilt1(table2array(T(:,"x")));
y = medfilt1(table2array(T(:,"y")));
%extract base values
base_x = medfilt1(table2array(T(:,"xFulc")));
base_y = medfilt1(table2array(T(:,"yFulc")));
continuum_pix = sqrt((y(1)-base_y(1))^2);
%normalise data to (0,0) origin
x = x - x(1);
y = y - y(1);
figure(2)
hold on
y = -y;
plot (x,y)
%scale pixels into meters
cont_length = 0.77;
scale_fact = cont_length/continuum_pix;
x_m = (x.*scale_fact);
y_m = (y.*scale_fact);
%plot motion path on it's own
figure(1)
hold on
plot((x_m), (y_m));
grid on;

%%% FILE 2 %%%
%read in csv file for values
T = readtable('final_continuum_2.csv');
%extract x and y values respectivley
x = medfilt1(table2array(T(:,"x")));
y = medfilt1(table2array(T(:,"y")));
%extract base values
base_x = medfilt1(table2array(T(:,"xFulc")));
base_y = medfilt1(table2array(T(:,"yFulc")));
continuum_pix = sqrt((y(1)-base_y(1))^2);
%normalise data to (0,0) origin
x = x - x(1);
y = y - y(1);
figure(2)
hold on
y = -y;
plot (x,y)
%scale pixels into meters
cont_length = 0.77;
scale_fact = cont_length/continuum_pix;
fulc_x = 0;
fulc_y = 0;
x_m = (x.*scale_fact);
y_m = (y.*scale_fact);
%plot motion path on it's own
figure (1)
plot((x_m), (y_m));
xlabel('x (m)')
ylabel('y (m)')
legend('Test 1', 'Test 2')