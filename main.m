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

%% Third control method
[sysob_itae, sysob_cl_itae, sysobext_itae, K_itae, Kp_itae, Kext_itae, L_itae] = itae(A,B,C,D);

%% Stepinfo
% disp('sys1')
% stepinfo(out.sys1.Data, out.sys1.time)
% disp()
% 
% disp('syso1')
% stepinfo(out.syso1.Data, out.syso1.time)
% disp()
% 
% disp('sysco1')
% stepinfo(out.sysco1.Data, out.sysco1.time)
% disp()
% 
% disp('syscoi1')
% stepinfo(out.syscoi1.Data, out.syscoi1.time)
% disp()
% 
% disp('sys2')
% stepinfo(out.sys2.Data, out.sys2.time)
% disp()
% 
% disp('syso2')
% stepinfo(out.syso2.Data, out.syso2.time)
% disp()
% 
% disp('sysco2')
% stepinfo(out.sysco2.Data, out.sysco2.time)
% disp()
% 
% disp('syscoi2')
% stepinfo(out.syscoi2.Data, out.syscoi2.time)
% disp()
