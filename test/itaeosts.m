function [os, st] = itaeosts(Wn)
    %%Ball and Beam Plant
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

    itaeeq = [1 2.1*Wn 3.4*Wn^2 2.7*Wn^3 Wn^4];
    itaepol = roots(itaeeq);
    pole1 = itaepol(1);
    pole2 = itaepol(2);
    pole3 = itaepol(3);
    pole4 = itaepol(4);
    pole5 = real(pole3)*10;

    %Gain to place the eigenvalues in the poles desired
    K1 = place(A,B,itaepol);
   
    S = stepinfo(ss(A-B*K1, B, C, D));
    os = S.Overshoot;
    st = S.SettlingTime;
end