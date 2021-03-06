% %% Ball and Beam Plant
g = 9.8; %Gravity acceleration
m_B = 0.064;
m_b = 0.65;
R1 = 0.0254;
L = 0.425;
d = 0.12;
delta_2 = 0.2;
Km = 0.00767;
Ki = 0.00767;
Kg = 14;
Rm = 2.6;
Jb = 0.5*m_b*L^2;
n_motor = 0.69;
n_gearbox = 0.85;
n_total = n_motor + n_gearbox;

%% Ball and Beam Plant
A = [0 0 1 0 ; 
    0 0 0 1 ; 
    0 -(m_B*Jb*g+m_B^2*g*delta_2^2)/(Jb + m_B*delta_2^2)^2 -((Kg^2*Ki*Km*n_total)/(Rm*(Jb + m_B*delta_2^2))*(L^2)/(d^2)) 0 ;
    -(5*g)/7 0 0 0];
B = [0;0;(Kg*Ki*n_total)/(Rm*(Jb+m_B*delta_2^2))*(L/d);0];
C = [0 1 0 0];
D = 0;

sys = (A,B,C,D);

%% Ball and Beam Plant + IP

IP = 1.3; % Incerteza paramétrica

gip = 9.8*IP; %Gravity acceleration
m_Bip = 0.064*IP;
m_bip = 0.65*IP;
R1ip = 0.0254*IP;
Lip = 0.425*IP;
dip = 0.12*IP;
delta_2ip = 0.2*IP;
Kmip = 0.00767*IP;
Kiip = 0.00767*IP;
Kgip = 14*IP;
Rmip = 2.6*IP;
Jbip = 0.5*m_bip*Lip^2*IP;
n_motorip = 0.69*IP;
n_gearboxip = 0.85*IP;
n_total = n_motorip + n_gearboxip;


%% Ball and Beam Plant IP
Aip = [0 0 1 0 ; 
    0 0 0 1 ; 
    0 -(m_Bip*Jbip*gip+m_Bip^2*gip*delta_2ip^2)/(Jbip + m_Bip*delta_2ip^2)^2 -((Kgip^2*Kiip*Kmip*n_total)/(Rmip*(Jbip + m_Bip*delta_2ip^2))*(Lip^2)/(dip^2)) 0 ;
    -(5*gip)/7 0 0 0];
Bip = [0;0;(Kgip*Kiip*n_total)/(Rmip*(Jbip+m_Bip*delta_2ip^2))*(Lip/dip);0];
Cip = [0 1 0 0];
Dip = 0;


sysIP = ss(Aip,Bip,Cip,Dip);

%% Observability
ov = obsv(A,C);

%% Controllability
ct = ctrb(A,B);

%% First control method
[sysob_pole, sysob_cl_pole, sysobext_pole, K_pole, Kp_pole, Kext_pole, L_pole] = pole_placement(A,B,C,D);

%% Second control method
[sysob_itae, sysob_cl_itae, sysobext_itae, K_itae, Kp_itae, Kext_itae, L_itae] = itae(A,B,C,D);

%% Third control method
Q =  [100 0 0 0; 
0 1 0 0;
0 0 1 0;
0 0 0 1]; % Más rápido a cambio de gastar más energía 
Qext =[100 0 0 0 0; 
0 1 0 0 0;
0 0 1 0 0;
0 0 0 1 0;
0 0 0 0 2]; %Más rápido a cambio de gastar más energía 

R = 0.0001; % Ahorrar energía a cambio de que sea más lento
[sysob_lqr, sysob_cl_lqr, sysobext_lqr, K_lqr, Kp_lqr, Kext_lqr, L_lqr] = mylqr(A,B,C,D,Q,Qext,R);


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
