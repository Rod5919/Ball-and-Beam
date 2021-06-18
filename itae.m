function [sysobs_itae, sysobs_cl_itae, sysobsext_itae, K_itae, Kp_itae, Kext_itae, L_itae] = itae(A,B,C,D)
%itae - Function created by: Jaime Condori & Sergio Fernandez
%
% Syntax: [sys, sysobs, k1, L1] = itae(A,B,C,D)
%
% A,B,C,D is thew state space
% ITAE function which returns:
% System without control
% Observable system 
% Observable system 
% Observable system  + integral error
% Observable system  + integral error + gain
% ITAE retroalimentation gain
% Precompensation gain
% ITAE Observable vector
    
% Frecuencia natural
    Wn = 7;

    itaeeq = [1 2.1*Wn 3.4*Wn^2 2.7*Wn^3 Wn^4];
    itaepol = roots(itaeeq);
    pole1 = itaepol(1);
    pole2 = itaepol(2);
    pole3 = itaepol(3);
    pole4 = itaepol(4);
    pole5 = real(pole3)*10;

    %Gain to place the eigenvalues in the poles desired
    K_itae = place(A,B,itaepol);

    %Gain to deal with Static Error
    Kp_itae = -inv(((C-D*K_itae)*inv((A-B*K_itae))*B-D));

    %PUC (Plant Under Control)
    %% Control de: realimentación + error estático
    n = rank(A);
    Aext = [A zeros(n,1); -C 0];
    Bext = [B; 0];
    Cext = [C 0];

    %Gain for the plant with integral error
    Kext_itae = place(Aext,Bext,[pole1,pole2,pole3,pole4,pole5]);

    %Without Matlab "1st Observer" (poles wanted for the observer)
    pole_ob11 = real(pole1)*10;
    pole_ob12 = real(pole1)*10+1;
    pole_ob13 = real(pole1)*10+2;
    pole_ob14 = real(pole1)*10+3; 

    %Gain to confirm the followed manual process
    L_itae = place(A',C',[pole_ob11 pole_ob12 pole_ob13 pole_ob14]);
    L_itae = L_itae';

    %State Space for the 1st observer
    Aob1 = A-L_itae*C;
    Bob1 = [B-L_itae*D L_itae];
    Cob1 = eye(4);
    Dob1 = zeros(rank(A),2);
    sysobs_itae = ss(Aob1,Bob1,Cob1,Dob1);    
    %disp(Dob)
    
    %Control + Observer 
    Aobcl1 = A-L_itae*C-B*K_itae+L_itae*D*K_itae;
    Bobcl1 = [B-L_itae*D L_itae];
    Cobcl1 = -K_itae;
    Dobcl1 = [1 0];
    sysobs_cl_itae = ss(Aobcl1,Bobcl1,Cobcl1,Dobcl1);    
    
    %Control + Observer + integral error
    K11 = Kext_itae(1:rank(A));
    Ke = Kext_itae(rank(A)+1);
    Aobcl_1 = [Aob1-B*K11+L_itae*D*K11 -B*Ke ; zeros(1,n) 0];
    Bobcl_1 = [zeros(n,1) L_itae ; 1 -1];
    Cobcl_1 = -Kext_itae;
    Dobcl_1 = [0 0];
    sysobsext_itae = ss(Aobcl_1,Bobcl_1,Cobcl_1,Dobcl_1);    
end