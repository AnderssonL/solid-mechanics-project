function [F_B, Nf, Nb, M_b] = broms_hjulkrafter(df, db, h1, h, m, Cd, p, v, a, r, g)
F_L = 1/2 * Cd * p * v^2;
F_B = m * a - F_L;
höjd = h1 + h;
billängd = (df + db)

Nf = ((db * m * g + h * m * a - (höjd) * F_L)) / billängd;

Nb = (df * m * g + (höjd) * F_L - h * m * a) / billängd;

M_b = F_B * r / 2; % tot-drivkraften * radien av hjulet / 2 (eftersom det blir på vardera hjul)
end