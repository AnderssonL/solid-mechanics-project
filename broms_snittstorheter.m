function [y_vec, Tyx, Tyz, N_kraft, Mx, My, Fb] = broms_snittstorheter(L, bb, b1, bd, N_points, Nb, m, a_broms, Cd, A, rho, v, r_hjul, r_broms)
% BROMS_SNITTSTORHETER Beräknar snittkrafter och bromskraft internt
%
% Inputs:
%   ...geometri...
%   Cd, A, rho, v - För beräkning av luftmotstånd
%   m, a_broms    - För beräkning av tröghetskraft

    % --- 1. Beräkna krafter internt ---
    
    % Beräkna Luftmotstånd: F_luft = 0.5 * rho * A * Cd * v^2
    F_luft = 0.5 * rho * A * Cd * v^2;
    
    % Total kraft vid vägbanan (Tröghet + Luft)
    F_total_road = (m * a_broms) - F_luft; % samma som F_b i broms_hjulkrafter
    
    % Moment från hjulet
    M_wheel = F_total_road * r_hjul;
    
    % Bromskraft på skivan (Klämkraft/Skjuv)
    Fb = M_wheel / r_broms;
    
    % Reaktionskrafter i lagren (X-led)
    Ryx = (Fb * (b1 - bb)) / (L - 2*b1);
    Rix = Fb - Ryx;

    % --- 2. Initiera vektorer ---
    y_vec = linspace(0, L, N_points);
    Tyx = zeros(1, N_points); Tyz = zeros(1, N_points);
    N_kraft = zeros(1, N_points); Mx = zeros(1, N_points); My = zeros(1, N_points);

    % --- 3. Loopa genom intervallen ---
    for i = 1:N_points
        y = y_vec(i);
        
        if y >= 0 && y < bb
            Tyx(i) = 0;
            Tyz(i) = -Nb;
            Mx(i)  = -Nb * y;
            My(i)  = -M_wheel;    
            
        elseif y >= bb && y < b1
            Tyx(i) = -Fb;         
            Tyz(i) = -Nb;
            Mx(i)  = -Nb * y;
            My(i)  = 0;           

        elseif y >= b1 && y < bd
            Tyx(i) = -Fb + Rix;
            Tyz(i) = 0;
            Mx(i)  = -Nb * b1;
            My(i)  = 0;

        elseif y >= bd && y < (L - b1)
            Tyx(i) = -Ryx;
            Tyz(i) = 0;
            Mx(i)  = -Nb * b1;
            My(i)  = 0;

        elseif y >= (L - b1) && y < (L - bb)
            Tyx(i) = 0;
            Tyz(i) = Nb;
            Mx(i)  = Nb * (y - L);
            My(i)  = 0;

        elseif y >= (L - bb) && y <= L
            Tyx(i) = 0;
            Tyz(i) = Nb;
            Mx(i)  = Nb * (y - L);
            My(i)  = 0;
        end
    end
end



