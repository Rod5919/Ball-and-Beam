%% Ball and Beam Plant
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

A = [0 0 1 0 ; 
    0 0 0 1 ; 
    0 -(m_B*Jb*g+m_B^2*g*delta_2^2)/(Jb + m_B*delta_2^2)^2 -((Kg^2*Ki*Km*n_total)/(Rm*(Jb + m_B*delta_2^2))*(L^2)/(d^2)) 0 ;
    -(5*g)/7 0 0 0];
B = [0;0;(Kg*Ki*n_total)/(Rm*(Jb+m_B*delta_2^2))*(L/d);0];
C = [0 1 0 0];
D = 0;

% Incerteza Paramétrica
IP = 1;

state_space = ss(A,B,C,D);
%%Pole placement implementation
%Especificaciones del control
PO = 4;
ts = 0.7;
%Polos dominantes
Ecl = abs(log(PO/100))/sqrt(pi^2+(log(PO/100))^2);
Wncl = 4/(Ecl*ts);

pole1 = -Ecl*Wncl+Wncl*sqrt(Ecl^2-1);
pole2 = -Ecl*Wncl-Wncl*sqrt(Ecl^2-1);
pole3 = real(pole1)*10;
pole4 = real(pole1)*10+1;
pole5 = real(pole1)*10+2;
%Gain to place the eigenvalues in the poles desired
K1 = place(A,B,[pole1 pole2 pole3 pole4]);

%Gain to deal with Static Error
Kp1 = -inv(((C-D*K1)*inv((A-B*K1))*B-D));
%PUC (Plant Under Control)

% Control de: realimentación + error estático
n = rank(A);
Aext = [A zeros(n,1); -C 0];
Bext = [B; 0];
Cext = [C 0];

% Gain for the plant with integral error
Kext_1 = place(Aext,Bext,[pole1,pole2,pole3,pole4,pole5]);

% Without Matlab "1st Observer" (poles wanted for the observer)
pole_ob11 = real(pole1)*10;
pole_ob12 = real(pole1)*10+1;
pole_ob13 = real(pole1)*10+2;
pole_ob14 = real(pole1)*10+3; 

%Gain to confirm the followed manual process
L1 = place(A',C',[pole_ob11 pole_ob12 pole_ob13 pole_ob14]);
L1 = L1';

% State Space for the 1st observer
Aob1 = A-L1*C;
Bob1 = [B-L1*D L1];
Cob1 = eye(4);
Dob1 = zeros(rank(A),2);
%disp(Dob)

% Control + Observer 
Aobcl1 = A-L1*C-B*K1+L1*D*K1;
Bobcl1 = [B-L1*D L1];
Cobcl1 = -K1;
Dobcl1 = [1 0];

%Control + Observer + integral error
K11 = Kext_1(1:rank(A));
Ke = Kext_1(rank(A)+1);
Aobcl_1 = [Aob1-B*K11+L1*D*K11 -B*Ke ; zeros(1,n) 0];
Bobcl_1 = [zeros(n,1) L1 ; 1 -1];
Cobcl_1 = -Kext_1;
Dobcl_1 = [0 0];

%% LQR implementation
Q =  [1 0 0 0; 
      0 1 0 0;
      0 0 1 0;
      0 0 0 1]; % Más rápido a cambio de gastar más energía 
R = 0.0001; % Ahorrar energía a cambio de que sea más lento
K_lqr = lqr(A,B,Q,R);

% Gain to deal with Static Error
Kp_lqr = -inv(((C-D*K_lqr)*inv((A-B*K_lqr))*B-D));

% Control de: realimentación + error estático
Qext =[1 0 0 0 0; 
       0 1 0 0 0;
       0 0 1 0 0;
       0 0 0 1 0;
       0 0 0 0 5]; %Más rápido a cambio de gastar más energía 
   
R = 0.0001; %Ahorrar energía a cambio de que sea más lento

% Gain for the plant with integral error
Kext_lqr = lqr(Aext,Bext,Qext,R);
% A with close loop
Acl_lqr = A - B*K_lqr;

poles_lqr = eig(Acl_lqr);

% Gain for observers
L_lqr = place(A',C',[poles_lqr(1) poles_lqr(2) poles_lqr(3) poles_lqr(4)]);
L_lqr = L_lqr';

% State Space for the 1st observer
Aob_lqr = A-L_lqr*C;
Bob_lqr = [B-L_lqr*D L_lqr];
Cob_lqr = eye(4);
Dob_lqr = zeros(rank(A),2);

% Control + Observer 
Aobcl_lqr = A-L_lqr*C-B*K_lqr+L_lqr*D*K_lqr;
Bobcl_lqr = [B-L_lqr*D L_lqr];
Cobcl_lqr = -K_lqr;
Dobcl_lqr = [1 0];

% Control + Observer + integral error
K11_lqr = Kext_lqr(1:rank(A));
Ke_lqr = Kext_lqr(rank(A)+1);
Aobcl_1lqr = [Aob_lqr-B*K11_lqr+L_lqr*D*K11_lqr -B*Ke_lqr ; zeros(1,n) 0];
Bobcl_1lqr = [zeros(n,1) L_lqr ; 1 -1];
Cobcl_1lqr = -Kext_lqr;
Dobcl_1lqr = [0 0];


%% ITAE implementation
Wn_itae = 7;
itaeeq = [1 2.1*Wn_itae 3.4*Wn_itae^2 2.7*Wn_itae^3 Wn_itae^4];
itaepol = roots(itaeeq);
pole1_itae = itaepol(1);
pole2_itae = itaepol(2);
pole3_itae = itaepol(3);
pole4_itae = itaepol(4);
pole5_itae = real(pole3_itae)*10;

%Gain to place the eigenvalues in the poles desired
K_itae = place(A,B,itaepol);

%Gain to deal with Static Error
Kp_itae = -inv(((C-D*K_itae)*inv((A-B*K_itae))*B-D));
%PUC (Plant Under Control)

% Control de: realimentación + error estático
n = rank(A);
Aext_itae = [A zeros(n,1); -C 0];
Bext_itae = [B; 0];
Cext_itae = [C 0];

% Gain for the plant with integral error
Kext_itae = place(Aext,Bext,[pole1_itae,pole2_itae,pole3_itae,pole4_itae,pole5_itae]);

% Without Matlab "1st Observer" (poles wanted for the observer)
pole_ob11_itae = real(pole1_itae)*10;
pole_ob12_itae = real(pole1_itae)*10+1;
pole_ob13_itae = real(pole1_itae)*10+2;
pole_ob14_itae = real(pole1_itae)*10+3; 

% Gain to confirm the followed manual process
L_itae = place(A',C',[pole_ob11_itae pole_ob12_itae pole_ob13_itae pole_ob14_itae]);
L_itae = L_itae';

% State Space for the 1st observer
Aob_itae = A-L_itae*C;
Bob_itae = [B-L_itae*D L_itae];
Cob_itae = eye(4);
Dob_itae = zeros(rank(A),2);
%disp(Dob)

% Control + Observer 
Aobcl_itae = A-L_itae*C-B*K_itae+L_itae*D*K_itae;
Bobcl_itae = [B-L_itae*D L_itae];
Cobcl_itae = -K_itae;
Dobcl_itae = [1 0];

% Control + Observer + integral error
K11_itae = Kext_itae(1:rank(A));
Ke_itae = Kext_itae(rank(A)+1);
Aobcl_1itae = [Aob1_itae-B*K11_itae+L_itae*D*K11_itae -B*Ke_itae ; zeros(1,n) 0];
Bobcl_1itae = [zeros(n,1) L_itae ; 1 -1];
Cobcl_1itae = -Kext_itae;
Dobcl_1itae = [0 0];




