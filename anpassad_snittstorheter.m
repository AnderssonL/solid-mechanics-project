function [y_vec, Tyx, Tyz, N_vec, Mx, My, Mz] = anpassad_snittstorheter(fall, L, bb, b1, bd, N_points, r_hjul, r_drev, r_broms, Vbi, Hbi, R_ix, R_iy, R_iz, R_yx, R_yz, F_k, F_b)

    % Initiera vektorer
    y_vec = linspace(0, L, N_points);
    Tyx = zeros(1, N_points); 
    Tyz = zeros(1, N_points);
    N_vec = zeros(1, N_points);   
    Mx  = zeros(1, N_points); 
    My  = zeros(1, N_points); 
    Mz  = zeros(1, N_points); 

    % --- Mappa om input-namn till namnen i din loop ---
    Riz = R_iz;
    Rix = R_ix;
    Riy = R_iy;
    Ryz = R_yz;
    Ryx = R_yx;
    
    % Mbi är momentet vid balkens ände (y=0). Antas noll för fritt roterande hjulaxel-ände.
    Mbi = Hbi * r_hjul; 

    % --- Definiera last-variabler beroende på fall ---
    % Vi måste räkna ut vad F_driv, M_D, Mk, etc. ska vara
    % så att din loop ger rätt fysikaliskt resultat.

    % Förbered hjälpstorheter
    M_val_driv  = F_k * r_drev;    % Moment från kedjan
    M_val_broms = F_b * r_broms;   % Moment från bromsen (per sida)

    if fall == 1 % ACCELERATION
        % Din loop säger: Tyx = -F_driv/2.
        % Vid accel är kraften framåt (+). Alltså måste F_driv vara negativ här för att ge plus.
        F_driv_total_mag = (M_val_driv / r_hjul);
        F_driv = -F_driv_total_mag; 
        
        % Moment
        Mk = M_val_driv;           % Drevet driver (+)
        M_D = -M_val_driv / 2;     % Hjulet håller emot (-)
        
        % Drevkraft
        Fk = F_k;

        % Broms är 0
        F_broms = 0;
        Fb = 0;
        Mb = 0;

    elseif fall == 2 % BROMSNING
        % Din loop säger: Tyx = -F_driv/2.
        % Vid broms är kraften bakåt (-). Alltså måste F_driv vara positiv här för att ge minus.
        F_broms_total_mag = (M_val_broms * 2 / r_hjul);
        F_driv = F_broms_total_mag; 
        
        % Bromskrafter
        % Din loop: Tyx = ... + F_broms. Bromsoket håller emot bakåt (-).
        F_broms = -F_b;
        
        % Fb används i Mz = ... - Fb * bb. Fb ska vara magnituden (positiv).
        Fb = F_b;
        
        % Moment
        Mk = 0;
        Fk = 0;
        Mb = M_val_broms;          % Bromsmoment
        M_D = M_val_broms;         % Hjulet driver axeln (tröghet) (+)

    else % KURVTAGNING (Fall 3)
        % Liknar acceleration men med liten drivkraft (underhåll)
        F_driv_total_mag = (M_val_driv / r_hjul);
        F_driv = -F_driv_total_mag; % Negativ för att ge positiv framdrivning i din loop
        
        Mk = M_val_driv;
        M_D = -M_val_driv / 2;
        Fk = F_k;
        
        F_broms = 0;
        Fb = 0;
        Mb = 0;
    end

for i = 1:N_points
y = y_vec(i);

if y >= 0 && y < bb
    Tyz_int = -Vbi;
    Tyx_int = -F_driv/2;
    N = Hbi;
    Mx_int = Tyz_int * y - Mbi;
    My_int = M_D; 
    Mz_int = Tyx_int * y;
end
if y >= bb && y < b1
    Tyz_int = -Vbi;
    Tyx_int = -F_driv/2 + F_broms;
    N = Hbi;
    Mx_int = Tyz_int * y - Mbi;
    My_int = M_D - Mb; 
    Mz_int = Tyx_int * y - Fb * bb;
end
if y >= b1 && y < bd
    Tyz_int = -Vbi - Riz;
    Tyx_int = -F_driv/2 + F_broms - Rix;
    N = Hbi - Riy;
    Mx_int = Tyz_int * y - Mbi + Riz * b1;
    My_int = M_D - Mb;
    Mz_int = Tyx_int * y - Fb * bb + Rix * b1;
end
if y >= bd && y < (L-b1)
    Tyz_int = -Vbi - Riz;
    Tyx_int = -F_driv/2 + F_broms - Rix - Fk;
    N = Hbi - Riy;
    Mx_int = Tyz_int * y - Mbi + Riz * b1;
    My_int = M_D - Mb + Mk;
    Mz_int = Tyx_int * y - Fb * bb + Rix * b1 + Fk * bd;
end
if y >= (L-b1) && y < (L-bb)
    Tyz_int = -Vbi - Riz - Ryz;
    Tyx_int = -F_driv/2 + F_broms - Rix - Fk- Ryx;
    N = Hbi - Riy;
    Mx_int = Tyz_int * y - Mbi + Riz * b1 + Ryz * (L-b1);
    My_int = M_D - Mb + Mk;
    Mz_int = Tyx_int * y - Fb * bb + Rix * b1 + Fk * bd + Ryx * (L-b1);
end
if y >= (L-bb) && y < L
    Tyz_int = -Vbi - Riz - Ryz;
    Tyx_int = -F_driv/2 + 2* F_broms - Rix - Fk- Ryx;
    N = Hbi - Riy;
    Mx_int = Tyz_int * y - Mbi + Riz * b1 + Ryz * (L-b1);
    My_int = M_D - 2*Mb + Mk;
    Mz_int = Tyx_int * y - Fb * L + Rix * b1 + Fk * bd + Ryx * (L-b1);
end

Tyz(i) = Tyz_int;
Tyx(i) = Tyx_int;
N_vec(i) = N; % Store the normal force for the current iteration
Mx(i) = Mx_int;
My(i) = My_int; % Store the moment about the y-axis for the current iteration
Mz(i) = Mz_int; % Store the moment about the z-axis for the current iteration
end


end

