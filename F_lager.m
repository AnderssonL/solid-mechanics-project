function d = F_lager(H_bi, H_by, V_bi, V_by ,F_k,F_b)
syms R_yx R_ix R_yz R_iz R_iy
% Alla dessa måste defineras
F_d = 1;
L = 10;
B_1 = 0.1;
d_h = 1;
r_b = 1;
r_a=1;
r_d=1;
b_b= 1;
F_bi = F_b;
F_by = F_b;
b_1 = 4;

b_d=1;

ex = F_d+R_ix+R_yx+F_k-2*F_b;
ey = H_bi+H_by-R_iy;
ez = V_bi + V_by + R_iz+R_yz;
Mx = L/2*(V_bi-V_by)+d_h/2*(H_bi+H_by)+ (L/2-B_1)*(R_iz+R_yz);
My = -F_d * d_h/2-2*F_b*r_b +F_k*r_d;
Mz = L/2*(F_d/2-F_d/2)+(L/2-b_b)*(F_bi-F_by)+(L/2-b_1)*(R_yx-R_ix)-(L/2-b_d)*F_k;
eq1 = [ex ey ez Mx My Mz]  == [0 0 0 0 0 0];


d = solve(eq1,[R_yx R_ix R_yz R_iz R_iy]);