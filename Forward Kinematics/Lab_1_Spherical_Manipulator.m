disp('Spherical_RRP')
syms a1 a2 a3 a4 t1 t2 d3

%% Link lengths
a1 = 40;
a2 = 10;
a3 = 15;

%% D-H Parameters [theta, d, r, alpha, offset]1
% if prismatic joint: theta = theta, d = 0, offset = 1, after offset put the value of d
% if revolute joint: theta = 0,offset = 0, after offset put the value of theta
        
H0_1 = Link([0,a1,0,pi/2,0,0])
H0_1.qlim = pi/180*[-90,90]

H1_2 = Link([0,0,0,pi/2,0,pi/2])
H1_2.qlim = pi/180*[-90,90]

H2_3 = Link([0,0,0,0, 1,a2+a3])
H2_3.qlim = [0,10] 


Spherical_RRP = SerialLink([H0_1 H1_2 H2_3], 'name', 'Spherical_RRP')
Spherical_RRP.plot([0 0 0], 'workspace', [-15 15 -15 15 0 20])
Spherical_RRP.teach
