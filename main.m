%% Ball and Beam Plant
g = 9.8; %Gravity acceleration
mb = 0.65; 
R = 0.0254;
L = 0.425;
d = 0.12;
delta_2 = 0.2;
Km = 0.00767;
Ki = 0.00767;
Kg = 14;
Rm = 2.6;
Jb = 0.5;
n_motor = 0.69;
n_gearbox = 0.85;
n_total = n_motor + n_gearbox;

A = [0 0 1 0 ; 
    0 0 0 1 ; 
    0 -(mb*Jb*g+mb^2*g*delta_2^2)/(Jb + mb*delta_2^2)^2 -(Kg^2*Ki*Km*n_total)/(Rm*(Jb + mb*delta_2^2)*(L^2)/(d^2)) 0 ;
    -5*g/7 0 0 0];
B = [0;0;(Kg*Ki*n_total)/(Rm*(Jb+mb*delta_2^2)*((L/d)));0];
C = [0 1 0 0];
D = 0;

sys = ss(A,B,C,D);

%% First control method
[sysob_pole, sysob_cl_pole, sysobext_pole, K_pole, Kp_pole, Kext_pole, L_pole] = pole_placement(A,B,C,D);

%% Second control method
[sysob_itae, sysob_cl_itae, sysobext_itae, K_itae, Kp_itae, Kext_itae, L_itae] = itae(A,B,C,D);

%% Third control method
Q =  [1 0 0 0; 
0 1 0 0;
0 0 1 0;
0 0 0 1]; % Más rápido a cambio de gastar más energía 
Qext =[1 0 0 0 0; 
0 1 0 0 0;
0 0 1 0 0;
0 0 0 1 0;
0 0 0 0 5]; %Más rápido a cambio de gastar más energía 

R = 0.0001; % Ahorrar energía a cambio de que sea más lento
[sysob_lqr, sysob_cl_lqr, sysobext_lqr, K_lqr, Kp_lqr, Kext_lqr, L_lqr] = mylqr(A,B,C,D,Q,Qext,R);


% Stepinfo
%% State space
disp('State space without control')
stepinfo(out.sys1.Data, out.sys1.time)
disp()

%% Pole placement
disp('--------Pole placement--------')
disp('State Space for the 1st observer')
stepinfo(out.sysob_pole.Data, out.sysob_pole.time)
disp()

disp('Control + Observer')
stepinfo(out.sysob_cl_pole.Data, out.sysob_cl_pole.time)
disp()

disp('Control + Observer + integral error')
stepinfo(out.sysobext_pole.Data, out.sysobext_pole.time)
disp()

%% ITAE
disp('-------------ITAE-------------')
disp('State Space for the 1st observer')
stepinfo(out.sysob_itae.Data, out.sysob_itae.time)
disp()

disp('Control + Observer')
stepinfo(out.sysob_cl_itae.Data, out.sysob_cl_itae.time)
disp()

disp('Control + Observer + integral error')
stepinfo(out.sysobext_itae.Data, out.sysobext_itae.time)
disp()

%% LQR
disp('-------------LQR-------------')
disp('State Space for the 1st observer')
stepinfo(out.sysob_lqr.Data, out.sysob_lqr.time)
disp()

disp('Control + Observer')
stepinfo(out.sysob_cl_lqr.Data, out.sysob_cl_lqr.time)
disp()

disp('Control + Observer + integral error')
stepinfo(out.sysobext_lqr.Data, out.sysobext_lqr.time)
disp()