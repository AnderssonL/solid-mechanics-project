function [F_D, Nf, Nb, M_b] = accel_hjulkrafter(df, db, h1, h, m, Cd, p, v, a, r, g)
F_L = 1/2 * Cd * p * v^2;
F_D = F_L + m * a;
hojd = h1 + h;
billangd = (df + db);


Nf = (db * m * g - h * m * a - (hojd) * F_L) / billangd;

Nb = ((hojd) * F_L + df * m * g + h * m * a) / billangd;


M_b = (F_D) * r / 2; % tot-drivkraften * radien av hjulet / 2 (eftersom det blir på vardera hjul)
end