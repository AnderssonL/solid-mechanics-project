function [F_B, F_L, Nf, Nb, M_b] = broms_hjulkrafter(df, db, h1, h, m, Cd, p, v, a, r, g)
F_L = 1/2 * Cd * p * v^2;
F_B = m * a - F_L;
hojd = h1 + h;
billangd = (df + db);

Nf = ((db * m * g + h * m * a - (hojd) * F_L)) / billangd;

Nb = (df * m * g + (hojd) * F_L - h * m * a) / billangd;

M_b = F_B * r / 2; % tot-drivkraften * radien av hjulet / 2 (eftersom det blir på vardera hjul)
end