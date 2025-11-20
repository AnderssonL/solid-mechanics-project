function [y_vec, Tyx, Tyz, N_kraft, Mx, My] = accel_snittstorheter(L, bb, b1, bd, N_points, Nb, m, a_accel, Cd, A, rho, v, r_hjul, r_drev)
% ACCEL_SNITTSTORHETER Beräknar snittkrafter för acceleration
%
% Inputs:
%   m, a_accel    - Massa och acceleration
%   Cd, A, rho, v - För luftmotstånd
%   r_hjul, r_drev - Radier

    % --- Initiera vektorer ---
    y_vec = linspace(0, L, N_points);
    
    Tyx = zeros(1, N_points);     
    Tyz = zeros(1, N_points);     
    N_kraft = zeros(1, N_points); 
    Mx = zeros(1, N_points);      
    My = zeros(1, N_points);      

    % --- 1. Beräkna krafter internt ---
    
    % Luftmotstånd
    F_luft = 0.5 * rho * A * Cd * v^2;
    
    % Total drivkraft vid hjulet som krävs (Massa*a + Luftmotstånd)
    Fd_accel = m * a_accel + F_luft;

    % Drivmomentet på axeln
    Mk = Fd_accel * r_hjul; 
    
    % Kedjekraften Fk
    Fk = Mk / r_drev; 
    
    % Reaktionskraft Ryx (Högra lagret)
    Ryx = (Fk * (b1 - bd)) / (2*b1 - L);
    
    % Reaktionskraft Rix (Vänstra lagret)
    Rix = Ryx - Fk;

    % --- Loopa genom alla punkter ---
    for i = 1:N_points
        y = y_vec(i); 
        
        % Intervall 1: 0 till bb
        if y >= 0 && y < bb
            Tyx(i)     = 0;          
            Tyz(i)     = -Nb;        
            N_kraft(i) = 0;          
            Mx(i)      = -Nb * y;    
            My(i)      = 0;          
            
        % Intervall 2: bb till b1
        elseif y >= bb && y < b1
            Tyx(i)     = 0; 
            Tyz(i)     = -Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = -Nb * y; 
            My(i)      = 0; 

        % Intervall 3: b1 till bd
        elseif y >= b1 && y < bd
            Tyx(i)     = Rix;        
            Tyz(i)     = 0;          
            N_kraft(i) = 0; 
            Mx(i)      = -Nb * b1;   
            My(i)      = 0; 

        % Intervall 4: bd till L-b1
        elseif y >= bd && y < (L - b1)
            Tyx(i)     = Rix - Fk; 
            Tyz(i)     = 0; 
            N_kraft(i) = 0; 
            Mx(i)      = -Nb * b1; 
            My(i)      = Mk;         

        % Intervall 5: L-b1 till L-bb
        elseif y >= (L - b1) && y < (L - bb)
            Tyx(i)     = 0; 
            Tyz(i)     = Nb;         
            N_kraft(i) = 0; 
            Mx(i)      = Nb * (y - L); 
            My(i)      = Mk;         

        % Intervall 6: L-bb till L
        elseif y >= (L - bb) && y <= L
            Tyx(i)     = 0; 
            Tyz(i)     = Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = Nb * (y - L); 
            My(i)      = Mk; 
        end
    end
end
