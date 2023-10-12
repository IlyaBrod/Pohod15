m = 485.11e-3; %[kg] bigger segment in movement mass
l = 44e-3; %[m] segment length
h = 100e-3; %[m] mechanism height (distance from joint to base of the spring)
cosA = h/sqrt(h^2+l^2); %deducted distance of the mechanism diagonal (used for force projection)
I = 3.3298e+06*(1e-9); %[kg*m^2] inertia around joint rotation axe
accel = 10.46; %[rad/s^2] acceleration of the joint

Dx = 6e-3; %[m] maximal spring compression/elongation expected
g = 9.80665;


%% Static system

disp('Static system')
disp('Top spring [N/m] :')
k = m*g/Dx
disp('Bottom spring [N/m] :')
k/3.2

%% Dynamic system

disp('Dynamic system')
disp('Top spring [N/m] :')
k = (I*accel/l+m*g)/(Dx)
disp('Bottom spring [N/m] :')
k/3.2

%% Convert to daN/mm
disp('Top spring [daN/mm] :')
k*0.1/1e3
disp('Bottom spring [daN/mm] :')
k/3.2*0.1/1e3
