function [y_vec, Tyx, Tyz, N_kraft, Mx, My] = broms_snittstorheter(L, bb, b1, bd, N_points, Nb, Fb, Mbi)
% BROMS_SNITTSTORHETER Beräknar snittkrafter med 6 specifika intervall
%
% Inputs:
%   L        - Total längd (m)
%   bb       - Avstånd bb (m)
%   b1       - Avstånd b1 (m)
%   bd       - Avstånd bd (m)
%   N_points - Antal punkter
%   Nb, Fb, Mbi - Krafter och moment (lägg till fler här om du behöver)

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
            Tyx(i)     = 0;          % Fyll i rätt värde
            Tyz(i)     = -Nb;          % Fyll i rätt värde
            N_kraft(i) = 0;          % Fyll i rätt värde
            Mx(i)      = 0;          % Fyll i rätt värde
            My(i)      = -Nb * y;          % Fyll i rätt värde
            
        % --- Intervall 2: bb till b1 ---
        elseif y >= bb && y < b1
            Tyx(i)     = Fb; 
            Tyz(i)     = -Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = -Mbi; 
            My(i)      = -Nb * y; 

        % --- Intervall 3: b1 till bd ---
        elseif y >= b1 && y < bd
            Tyx(i)     = 0; 
            Tyz(i)     = 0; 
            N_kraft(i) = 0; 
            Mx(i)      = -Mbi; 
            My(i)      = -2*Nb * y - Nb * b1; 

        % --- Intervall 4: bd till L-b1 ---
        elseif y >= bd && y < (L - b1)
            Tyx(i)     = 0; 
            Tyz(i)     = 0; 
            N_kraft(i) = 0; 
            Mx(i)      = -Mbi; 
            My(i)      = -2*Nb * y - Nb * b1; 

        % --- Intervall 5: L-b1 till L-bb ---
        elseif y >= (L - b1) && y < (L - bb)
            Tyx(i)     = -Fb; 
            Tyz(i)     = Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = -Mbi; 
            My(i)      = Nb * L + Nb * y; 

        % --- Intervall 6: L-bb till L ---
        elseif y >= (L - bb) && y <= L
            Tyx(i)     = 0; 
            Tyz(i)     = Nb; 
            N_kraft(i) = 0; 
            Mx(i)      = 0; 
            My(i)      = Nb * L + Nb * y; 
        end
    end
end