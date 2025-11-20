function [y_vec, Tyx, Tyz, N_kraft, Mx, My] = accel_snittstorheter(L, bb, b1, bd, N_points, Nb, Rix, Fa, Mk)
% ACCEL_SNITTSTORHETER Beräknar snittkrafter för acceleration (6 intervall)
%
% Inputs:
%   L        - Total längd (m)
%   bb       - Avstånd bb (m)
%   b1       - Avstånd b1 (m)
%   bd       - Avstånd bd (m)
%   N_points - Antal punkter
%   Nb       - Lagerkraft/Normalkraft (N)
%   Rix      - Kraft Rix (N)
%   Fk       - Kraft Fk (N)
%   Mk       - Moment (Nm)

    % --- Initiera vektorer ---
    y_vec = linspace(0, L, N_points);
    
    Tyx = zeros(1, N_points);     
    Tyz = zeros(1, N_points);     
    N_kraft = zeros(1, N_points); 
    Mx = zeros(1, N_points);      
    My = zeros(1, N_points);      

    % --- Loopa genom alla punkter ---
    for i = 1:N_points
        y = y_vec(i); % Aktuell position
        
        % --- Intervall 1: 0 till bb ---
        if y >= 0 && y < bb
            Tyx(i)     = 0;          % Fyll i din formel
            Tyz(i)     = 0;          % Fyll i din formel
            N_kraft(i) = 0;          % Fyll i din formel
            Mx(i)      = 0;          % Fyll i din formel
            My(i)      = -Nb * y;          % Fyll i din formel
            
        % --- Intervall 2: bb till b1 ---
        elseif y >= bb && y < b1
            Tyx(i)     = 0; 
            Tyz(i)     = -Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = 0; 
            My(i)      = -Nb * y; 

        % --- Intervall 3: b1 till bd ---
        elseif y >= b1 && y < bd
            Tyx(i)     = Rix; 
            Tyz(i)     = 0; 
            N_kraft(i) = 0; 
            Mx(i)      = 0; 
            My(i)      = -Nb * b1; 

        % --- Intervall 4: bd till L-b1 ---
        elseif y >= bd && y < (L - b1)
            Tyx(i)     = Rix - Fk; 
            Tyz(i)     = 0; 
            N_kraft(i) = 0; 
            Mx(i)      = Mk; 
            My(i)      = -Nb * b1; 

        % --- Intervall 5: L-b1 till L-bb ---
        elseif y >= (L - b1) && y < (L - bb)
            Tyx(i)     = 0; 
            Tyz(i)     = Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = Mk; 
            My(i)      = -Nb * (y+L); 

        % --- Intervall 6: L-bb till L ---
        elseif y >= (L - bb) && y <= L
            Tyx(i)     = 0; 
            Tyz(i)     = Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = Mk; 
            My(i)      = -Nb * (y+L); 
        end
    end
end