function [sysob_lqr, sysob_cl_lqr, sysobext_lqr, K_lqr, Kp_lqr, Kext_lqr, L_lqr] = mylqr(A,B,C,D,Q,Qext,R)
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
    
    %% LQR implementation
%     Q =  [1 0 0 0; 
%       0 1 0 0;
%       0 0 1 0;
%       0 0 0 1]; % Más rápido a cambio de gastar más energía 
%     R = 0.0001; % Ahorrar energía a cambio de que sea más lento
    
    K_lqr = lqr(A,B,Q,R);

    % Gain to deal with Static Error
    Kp_lqr = -inv(((C-D*K_lqr)*inv((A-B*K_lqr))*B-D));

    % Control de: realimentación + error estático
%     Qext =[1 0 0 0 0; 
%            0 1 0 0 0;
%            0 0 1 0 0;
%            0 0 0 1 0;
%            0 0 0 0 5]; %Más rápido a cambio de gastar más energía 

    R = 0.0001; %Ahorrar energía a cambio de que sea más lento
    
    %% Control de: realimentación + error estático
    n = rank(A);
    Aext = [A zeros(n,1); -C 0];
    Bext = [B; 0];
    Cext = [C 0];
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
    sysob_lqr = ss(Aob_lqr,Bob_lqr,Cob_lqr,Dob_lqr);
    
    % Control + Observer 
    Aobcl_lqr = A-L_lqr*C-B*K_lqr+L_lqr*D*K_lqr;
    Bobcl_lqr = [B-L_lqr*D L_lqr];
    Cobcl_lqr = -K_lqr;
    Dobcl_lqr = [1 0];
    sysob_cl_lqr = ss(Aobcl_lqr,Bobcl_lqr,Cobcl_lqr,Dobcl_lqr);

    % Control + Observer + integral error
    K11_lqr = Kext_lqr(1:rank(A));
    Ke_lqr = Kext_lqr(rank(A)+1);
    Aobext_lqr = [Aob_lqr-B*K11_lqr+L_lqr*D*K11_lqr -B*Ke_lqr ; zeros(1,n) 0];
    Bobext_lqr = [zeros(n,1) L_lqr ; 1 -1];
    Cobext_lqr = -Kext_lqr;
    Dobext_lqr = [0 0]; 
    sysobext_lqr = ss(Aobext_lqr, Bobext_lqr, Cobext_lqr, Dobext_lqr);
end