function [F_D, Nf, Nb, M_b] = körning_hjulkrafter(df, db, h1, h, m, Cd, p, v, r, g)
F_L = 1/2 * Cd * p * v^2;
F_D = F_L;
höjd = h1 + h;
billängd = (df + db)

Nf = (db * m * g - (höjd) * F_L) / billängd;
Nb = ((höjd) * F_L + df * m * g) / billängd;

M_b = F_L * r / 2; % tot-drivkraften * radien av hjulet / 2 (eftersom det blir på vardera hjul)
end