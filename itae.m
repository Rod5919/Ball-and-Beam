function [sys, sysobs1, sysobs2, sysobs3, K1, Kp1, L1, Kext_1] = itae(A,B,C,D)
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
    sys = ss(A,B,C,D);
    Wn = 7;

    itaeeq = [1 2.1*Wn 3.4*Wn^2 2.7*Wn^3 Wn^4];
    itaepol = roots(itaeeq);
    pole1 = itaepol(1);
    pole2 = itaepol(2);
    pole3 = itaepol(3);
    pole4 = itaepol(4);
    pole5 = real(pole3)*10;

    %Gain to place the eigenvalues in the poles desired
    K1 = place(A,B,itaepol);

    %Gain to deal with Static Error
    Kp1 = -inv(((C-D*K1)*inv((A-B*K1))*B-D));
    %PUC (Plant Under Control)

    %% Control de: realimentación + error estático
    n = rank(A);
    Aext = [A zeros(n,1); -C 0];
    Bext = [B; 0];
    Cext = [C 0];

    %Gain for the plant with integral error
    Kext_1 = place(Aext,Bext,[pole1,pole2,pole3,pole4,pole5]);

    %Without Matlab "1st Observer" (poles wanted for the observer)
    pole_ob11 = real(pole1)*10;
    pole_ob12 = real(pole1)*10+1;
    pole_ob13 = real(pole1)*10+2;
    pole_ob14 = real(pole1)*10+3; 

    %Gain to confirm the followed manual process
    L1 = place(A',C',[pole_ob11 pole_ob12 pole_ob13 pole_ob14]);
    L1 = L1';

    %State Space for the 1st observer
    Aob1 = A-L1*C;
    Bob1 = [B-L1*D L1];
    Cob1 = eye(4);
    Dob1 = zeros(rank(A),2);
    sysobs1 = ss(Aob1,Bob1,Cob1,Dob1);    
    %disp(Dob)
    
    %Control + Observer 
    Aobcl1 = A-L1*C-B*K1+L1*D*K1;
    Bobcl1 = [B-L1*D L1];
    Cobcl1 = -K1;
    Dobcl1 = [1 0];
    sysobs2 = ss(Aobcl1,Bobcl1,Cobcl1,Dobcl1);    
    
    %Control + Observer + integral error
    K11 = Kext_1(1:rank(A));
    Ke = Kext_1(rank(A)+1);
    Aobcl_1 = [Aob1-B*K11+L1*D*K11 -B*Ke ; zeros(1,n) 0];
    Bobcl_1 = [zeros(n,1) L1 ; 1 -1];
    Cobcl_1 = -Kext_1;
    Dobcl_1 = [0 0];
    sysobs3 = ss(Aobcl_1,Bobcl_1,Cobcl_1,Dobcl_1);    
end