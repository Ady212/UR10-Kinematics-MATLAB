xc = 0;
yc = 800;
zc = 100;
D3 = 163.9;
D4 = 571.6;
L2 = 612.7;
rz = zc;

num = xc*D3+yc*sqrt((xc^2)+(yc^2)-(D3^2));
den = (xc^2)+(yc^2);

ct1= num/den;
if ct1 > 1
    ct1 = 1;
else
    if ct1 < -1
        ct1 = -1;
    end
end
st1= (D3-xc*ct1)/yc;

ry= yc*ct1 - xc*st1;

ct3 = ((ry^2)+(rz^2)-(L2^2)-(D4^2))/(2*L2*D4);
st3 = +-sqrt(1-ct3^2);

k1=L2+D4*cos(ct3);
k2= D4*st3;


ct2 = (rz*k1 - ry*k2)/((k1^2)+(k2^2));
st2 = (-ry -k2*ct2)/k1;


Angle1cos= acos(ct1);
Angle1sin= asin(st1);
Angle2cos= acos(ct2);
Angle2sin= asin(st2);
Angle3cos= acos(ct3);
Angle3sin= asin(st3);


% Task 1: Forward Kinematics Simulator for UR10 Manipulator
% This MATLAB script calculates and visualizes the forward kinematics of the UR10 arm.
% 
t = [Angle1cos Angle2cos Angle3cos deg2rad(-80)  -pi/2 0];

home = [ 0 0 0 0 0 0];

offset = [-90 90 0 90 0 0];
offsetrad = deg2rad(offset);

t2= [171.17 -93.83 -135.13 -41.44 87.40 -12.47];
t4= [171.15 -89.21 -132.81 -48.37 87.42 -12.47];
t1 = t4+offset;
t3 = t2+offset;
A1 = [cos(t(1)) -sin(t(1)) 0 0;
      sin(t(1))  cos(t(1)) 0 0;
         0          0      1 0;
         0          0      0 1;
     ];

A2 = [   1          0           0       0;
         0      cos(t(2))   -sin(t(2))  0;
         0      sin(t(2))    cos(t(2))  0;
         0          0           0       1;
     ];


A3 = [   1          0           0       0;
         0      cos(t(3))   -sin(t(3))  612.7*sin(t(3));
         0      sin(t(3))    cos(t(3))  612.7*(1-cos(t(3)));
         0          0           0       1;
     ];

A4 = [   1          0           0       0;
         0      cos(t(4))   -sin(t(4))  1184.3*sin(t(4));
         0      sin(t(4))    cos(t(4))  1184.3*(1-cos(t(4)));
         0          0           0       1;
     ];

A5 = [cos(t(5)) -sin(t(5)) 0  163.9*(1-cos(t(5)));
      sin(t(5))  cos(t(5)) 0 163.9*-sin(t(5));
         0          0      1 0;
         0          0      0 1;
     ];

A6 = [   1          0           0       0;
         0      cos(t(6))   -sin(t(6))  1300*sin(t(6));
         0      sin(t(6))    cos(t(6))  1300*(1-cos(t(6)));
         0          0           0       1;
     ];


p0 = [0; 0; 0; 1];
p1 = [0; 0; 128; 1];
p2 = [176; 0; 128; 1];
p3 = [176; 0; 740.70;  1];
p4 = [12.1; 0; 740.70; 1];
p5 = [12.1; 0; 1312.30; 1];
p6 = [163.9; 0; 1312.30; 1];
p7 = [163.9; 0; 1428; 1];
p8 = [256.1; 0; 1428; 1];


%% Apply transformations on Home position
q0 = (p0 - [0; 0; 128; 0]);
q1 = (p1 - [0; 0; 128; 0]);
q2 = A1*(p2 - [0; 0; 128; 0]);
q3 = A1*A2*(p3 - [0; 0; 128; 0]);
q4 = A1*A2*(p4 - [0; 0; 128; 0]);
q5 = A1*A2*A3*A4*(p5 - [0; 0; 128; 0]);
q6 = A1*A2*A3*A4*A5*(p6 - [0; 0; 128; 0]);
q7 = A1*A2*A3*A4*A5*A6*(p7 - [0; 0; 128; 0]);
q8 = A1*A2*A3*A4*A5*A6*(p8 - [0; 0; 128; 0]);

%% Final Configuration of the manipulator

homePositionX = [double(p0(1)) double(p1(1)) double(p2(1)) double(p3(1)) double(p4(1)) double(p5(1)) double(p6(1)) double(p7(1)) double(p8(1))];
homePositionY = [double(p0(2)) double(p1(2)) double(p2(2)) double(p3(2)) double(p4(2)) double(p5(2)) double(p6(2)) double(p7(2)) double(p8(2))];
homePositionZ = [double(p0(3)) double(p1(3)) double(p2(3)) double(p3(3)) double(p4(3)) double(p5(3)) double(p6(3)) double(p7(3)) double(p8(3))];

finalPositionX = [ double(q0(1)) double(q1(1)) double(q2(1)) double(q3(1)) double(q4(1)) double(q5(1)) double(q6(1)) double(q7(1)) double(q8(1))];
finalPositionY = [ double(q0(2)) double(q1(2)) double(q2(2)) double(q3(2)) double(q4(2)) double(q5(2)) double(q6(2)) double(q7(2)) double(q8(2))];
finalPositionZ = [ double(q0(3)) double(q1(3)) double(q2(3)) double(q3(3)) double(q4(3)) double(q5(3)) double(q6(3)) double(q7(3)) double(q8(3))] + 128;


%% Plot
figure
% Create plot3 of homePositionX, homePositionY, and homePositionZ
plot3(homePositionX,homePositionY,homePositionZ,'Marker','o','DisplayName','homePositionZ');

hold on
% Create plot3 of finalPositionX, finalPositionY, and finalPositionZ
plot3(finalPositionX,finalPositionY,finalPositionZ,'Marker','o','DisplayName','finalPositionZ');
hold off

legend

axis([-1000 1700 -1000 1700 0 1500])
grid on 

send2python(home,1)

pause(5)

send2python(deg2rad(t1),1)

pause(5)

send2python(deg2rad(t3),0)

pause(5)

send2python(deg2rad(t1),0)

pause(5)

send2python(home,0)

pause(5)

send2python(t,1)

pause(5)

send2python(home,0)