function [V_bi, V_fi, V_by, V_fy, H_bi, H_fi, H_by, H_fy, M_b, F_D, V_i, V_y, H_i, H_y] = kurvtagning_hjulkrafter(df, db, h1, h, m, Cd, p, v, R, L, r, g)

% --- Grundläggande krafter ---
F_L = 1/2 * Cd * p * v^2;
F_D = F_L; 
F_C = m * v^2 / R; 

% --- Totala normalkrafter per axel (som vid körning rakt fram) ---
axelavstand = df + db;
Nf = (db * m * g - (h1 + h) * F_L) / axelavstand;
Nb = ((h1 + h) * F_L + df * m * g) / axelavstand;

% --- Normalkrafter för respektive hjul (V) ---
% Fördelning av axel-normalkraft pga moment från centrifugalkraften
V_fi = Nf * (m * g * L - 2 * F_C * h) / (2 * m * g * L);
V_fy = Nf * (m * g * L + 2 * F_C * h) / (2 * m * g * L);
V_bi = Nb * (m * g * L - 2 * F_C * h) / (2 * m * g * L);
V_by = Nb * (m * g * L + 2 * F_C * h) / (2 * m * g * L);

V_i = V_bi + V_fi;
V_y = V_by + V_fy;
% --- Horisontella sidokrafter på hjulen (H) ---
% Fördelning av totala centrifugalkraften på fram- och bakaxel, sedan vidare till respektive hjul i proportion till deras normalkraft.
H_fi = (V_fi / (V_fi + V_fy)) * (F_C * db / axelavstand);
H_fy = (V_fy / (V_fi + V_fy)) * (F_C * db / axelavstand);
H_bi = (V_bi / (V_bi + V_by)) * (F_C * df / axelavstand);
H_by = (V_by / (V_bi + V_by)) * (F_C * df / axelavstand);

H_i = H_bi + H_fi;
H_y = H_by + H_fy;

% --- Drivmoment per bakhjul ---
M_b = F_D * r / 2;


end

