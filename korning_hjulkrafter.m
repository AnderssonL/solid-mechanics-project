function [F_D, Nf, Nb, M_b] = korning_hjulkrafter(df, db, h1, h, m, Cd, p, v, r, g)
F_L = 1/2 * Cd * p * v^2;
F_D = F_L;
hojd = h1 + h;
billangd = (df + db);

Nf = (db * m * g - (hojd) * F_L) / billangd;
Nb = ((hojd) * F_L + df * m * g) / billangd;

M_b = F_L * r / 2; % tot-drivkraften * radien av hjulet / 2 (eftersom det blir på vardera hjul)
end