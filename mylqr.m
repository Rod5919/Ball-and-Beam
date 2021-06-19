function [sysob, sysob_cl, sysobext, K, Kp, Kext, L] = mylqr(A,B,C,D,Q,Qext,R)
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
    
    K = lqr(A,B,Q,R);

    % Gain to deal with Static Error
    Kp = -inv(((C-D*K)*inv((A-B*K))*B-D));

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
    Kext = lqr(Aext,Bext,Qext,R);
    % A with close loop
    Acl = A - B*K;

    poles = eig(Acl);
    % Gain for observers
    L = place(A',C',[poles(1)*10 poles(2)*10 poles(3)*10 poles(4)*10]);
    L = L';

    % State Space for the 1st observer
    Aob = A-L*C;
    Bob = [B-L*D L];
    Cob = eye(4);
    Dob = zeros(rank(A),2);
    sysob = ss(Aob,Bob,Cob,Dob);
    
    % Control + Observer 
    Aobcl = A-L*C-B*K+L*D*K;
    Bobcl = [B-L*D L];
    Cobcl = -K;
    Dobcl = [1 0];
    sysob_cl = ss(Aobcl,Bobcl,Cobcl,Dobcl);

    % Control + Observer + integral error
    K11 = Kext(1:rank(A));
    Ke = Kext(rank(A)+1);
    Aobext = [Aob-B*K11+L*D*K11 -B*Ke ; zeros(1,n) 0];
    Bobext = [zeros(n,1) L ; 1 -1];
    Cobext = -Kext;
    Dobext = [0 0]; 
    sysobext = ss(Aobext, Bobext, Cobext, Dobext);
end