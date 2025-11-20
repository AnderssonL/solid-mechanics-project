function [y_vec, Tyx, Tyz, N_kraft, Mx, My] = kurvtagning_snittstorheter(L, bb, b1, bd, N_points, Vi, Vy, Riz, Hi, Hy)
% KURVTAGNING_SNITTSTORHETER Beräknar snittkrafter för kurvtagning (6 intervall)
%
% Inputs:
%   L        - Total längd (m)
%   bb       - Avstånd bb (m)
%   b1       - Avstånd b1 (m)
%   bd       - Avstånd bd (m)
%   N_points - Antal punkter
%   Vi       - Kraft Inre hjul (N)
%   Vy       - Kraft Yttre hjul (N)
%   Riz      - Reaktionskraft Z (N)
%   Hi       - Normalkraft Inre (N)
%   Hy       - Normalkraft Yttre (N)

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
            Tyz(i)     = -Vi;          % Fyll i din formel
            N_kraft(i) = Hi;          % Fyll i din formel
            Mx(i)      = -Vi * y;          % Fyll i din formel
            My(i)      = 0;          % Fyll i din formel
            
        % --- Intervall 2: bb till b1 ---
        elseif y >= bb && y < b1
            Tyx(i)     = 0; 
            Tyz(i)     = -Vi; 
            N_kraft(i) = Hi; 
            Mx(i)      = -Vi * y; 
            My(i)      = 0; 

        % --- Intervall 3: b1 till bd ---
        elseif y >= b1 && y < bd
            Tyx(i)     = 0; 
            Tyz(i)     = -Vi - Riz; 
            N_kraft(i) = Hy; 
            Mx(i)      = Riz * (b1-y) -Vi * y; 
            My(i)      = 0; 

        % --- Intervall 4: bd till L-b1 ---
        elseif y >= bd && y < (L - b1)
            Tyx(i)     = 0; 
            Tyz(i)     = -Vi - Riz; 
            N_kraft(i) = Hy; 
            Mx(i)      = Riz * (b1-y) -Vi * y; 
            My(i)      = 0; 

        % --- Intervall 5: L-b1 till L-bb ---
        elseif y >= (L - b1) && y < (L - bb)
            Tyx(i)     = 0; 
            Tyz(i)     = -Vy; 
            N_kraft(i) = Hy; 
            Mx(i)      = Riz * b1 + Ryz * (L -bi) + Vy * y; 
            My(i)      = 0; 

        % --- Intervall 6: L-bb till L ---
        elseif y >= (L - bb) && y <= L
            Tyx(i)     = 0; 
            Tyz(i)     = -Vy; 
            N_kraft(i) = Hy; 
            Mx(i)      = Riz * b1 + Ryz * (L -bi) + Vy * y; 
            My(i)      = 0; 
        end
    end
end