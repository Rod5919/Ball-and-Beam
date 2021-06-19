%% Ball and Beam Plant + IP
g_ip = 9.8*1.05;
mb_ip = 0.65*1.05; 
R_ip = 0.0254*1.05;
L_ip = 0.425*1.05;
d_ip = 0.12*1.05;
delta_2_ip = 0.2*1.05;
Km_ip = 0.00767*1.05;
Ki_ip = 0.00767*1.05;
Kg_ip = 14*1.05;
Rm_ip = 2.6*1.05;
Jb_ip = 0.5*1.05;
n_motor_ip = 0.69*1.05;
n_gearbox_ip = 0.85*1.05;
n_total_ip = n_motor + n_gearbox;

%% State space
A_ip = [0 0 1 0 ; 
    0 0 0 1 ; 
    0 -(mb*Jb*g+mb^2*g*delta_2^2)/(Jb + mb*delta_2^2)^2 -(Kg^2*Ki*Km*n_total)/(Rm*(Jb + mb*delta_2^2)*(L^2)/(d^2)) 0 ;
    -5*g/7 0 0 0];
B_ip = [0;0;(Kg*Ki*n_total)/(Rm*(Jb+mb*delta_2^2)*((L/d)));0];
C_ip = [0 1 0 0];
D_ip = 0;

sys_ip = ss(A_ip,B_ip,C_ip,D_ip);

%% Observability
ov_ip = obsv(A_ip,C_ip);

%% Controllability
ct_ip = ctrb(A_ip,B_ip);

%% First control method
[sysob_pole_ip, sysob_cl_pole_ip, sysobext_pole_ip, K_pole_ip, Kp_pole_ip, Kext_pole_ip, L_pole_ip] = pole_placement(A_ip,B_ip,C_ip,D_ip);

%% Second control method
[sysob_itae_ip, sysob_cl_itae_ip, sysobext_itae_ip, K_itae_ip, Kp_itae_ip, Kext_itae_ip, L_itae_ip] = itae(A_ip,B_ip,C_ip,D_ip);

%% Third control method
Q_ip =  [1 0 0 0; 
0 1 0 0;
0 0 1 0;
0 0 0 1]; % Más rápido a cambio de gastar más energía 
Qext_ip =[1 0 0 0 0; 
0 1 0 0 0;
0 0 1 0 0;
0 0 0 1 0;
0 0 0 0 5]; %Más rápido a cambio de gastar más energía 

R_ip = 0.0001; % Ahorrar energía a cambio de que sea más lento
[sysob_lqr_ip, sysob_cl_lqr_ip, sysobext_lqr_ip, K_lqr_ip, Kp_lqr_ip, Kext_lqr_ip, L_lqr_ip] = mylqr(A_ip,B_ip,C_ip,D_ip,Q,Qext,R);


% Stepinfo
%% State space
disp('State space without control')
step((L/2)*sys)
disp('')

%% Pole placement
disp('--------Pole placement--------')
disp('State Space for the 1st observer')
stepinfo(out.pole_obs.Data, out.pole_obs.time)
disp('')

disp('Control + Observer')
stepinfo(out.pole_obs.Data, out.pole_obs.time)
disp('')

disp('Control + Observer + integral error')
stepinfo(out.pole_obs.Data, out.pole_obs.time)
disp('')

%% ITAE
disp('-------------ITAE-------------')
disp('State Space for the 1st observer')
stepinfo(out.itae_obs.Data, out.itae_obs.time)
disp('')

disp('Control + Observer')
stepinfo(out.itae_obs.Data, out.itae_obs.time)
disp('')

disp('Control + Observer + integral error')
stepinfo(out.itae_obs.Data, out.itae_obs.time)
disp('')

%% LQR
disp('-------------LQR-------------')
disp('State Space for the 1st observer')
stepinfo(out.lqr_obs.Data, out.lqr_obs.time)
disp('')

disp('Control + Observer')
stepinfo(out.lqr_obs.Data, out.lqr_obs.time)
disp('')

disp('Control + Observer + integral error')
stepinfo(out.lqr_obs.Data, out.lqr_obs.time)
disp('')
