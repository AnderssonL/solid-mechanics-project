function [R_yx, R_ix, R_yz, R_iz, R_iy, F_k, F_b] = F_lager(m, L, b_1, d_h, r_b, r_d, b_b, b_d, a1, a2, Cd, rho_luft, v, styrVariabel, H_bi, H_by, V_bi, V_by, F_d)
syms R_yx R_ix R_yz R_iz R_iy

% Beräkna F_d och F_b respektive

F_luft=1/2*Cd*rho_luft*v^2;

%Styrvariabel används för att välja lastfallen. Vi har ingen kraft på drevet när vi bromsar till exempel.

if styrVariabel==1 %Acceleration konstant acceleration
    F_k=((m*a1+F_luft)*d_h/2)/r_d;
    F_b=0;

end

if styrVariabel==2 %Bromsning med konstant retardation
    F_k=0;
    F_b=((m*a2+F_luft)*d_h/2)/(r_b*2);

end

if styrVariabel==3 %Kurvtagning
    F_k=(F_luft*d_h/2)/r_d;
    F_b=0;

end

%Kraftekvationer

ex = F_d + R_ix + R_yx + F_k-2 * F_b == 0;

ey = H_bi + H_by - R_iy == 0;

ez = V_bi + V_by + R_iz + R_yz == 0;

%Momentekvationer
Mx = (L/2)*(V_bi-V_by) + (d_h/2)*(H_bi+H_by) + (L/2-b_1)*(R_iz-R_yz) == 0;

%My = (-F_d*d_h/2) - 2*F_b*r_b + F_k*r_d == 0;

Mz = (L/2)*(F_d/2-F_d/2) + (L/2-b_b)*(F_b-F_b) + (L/2-b_1)*(R_yx-R_ix) + (L/2-b_d)*(-F_k) == 0;

sol = solve([ex,ey,ez,Mx,Mz],[R_yx, R_ix, R_yz, R_iz, R_iy]);

R_yx = double(sol.R_yx);
R_ix = double(sol.R_ix);
R_yz = double(sol.R_yz);
R_iz = double(sol.R_iz);

R_iy = double(sol.R_iy);
