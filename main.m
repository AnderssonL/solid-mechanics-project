%% Vehicle parameters
% Värdena är tagna från "Typiska värden" i Tabell 2 i projektbeskrivningen.

m = 150;            % Fordonets vikt inkl. förare [kg]
df = 0.8;           % Avstånd framaxel till tyngdpunkt [m] (600-1000 mm)
db = 0.3;           % Avstånd bakaxel till tyngdpunkt [m] (200-400 mm)
hTP = 0.45;           % Tyngdpunktens vertikala position [m] (30-60 cm)
hFL = 0.2;           % Avstånd tyngdpunkt till luftmotståndets verkningslinje [m] (15-25 cm)
h_luft = hTP+ hFL;    % Höjd från marken till luftmotståndets angreppspunkt [m]
A_front = 0.5;      % Fordonets frontarea [m^2]
Cd = 0.3;           % Luftmotståndskoefficient (c i PM)
dh = 0.3;           % Hjuldiameter [m] (240-400 mm)
r_hjul = dh / 2;    % Hjulradie [m]
r_drev = 0.05;      % Drevets radie [m] 
r_broms = 0.09;     % Bromsskivans radie [m]


a1 = 6;             % Acceleration i accelerationslastfallet [m/s^2]
a2 = -15;           % Max retardation (värdet är positivt) [m/s^2]
v_max_kmh = 100;    % Maxfart (rakt fram) [km/h] (70-120)
v_max_ms = v_max_kmh / 3.6; % Konvertera maxfart till [m/s]
gamma = 0.8;        % Faktor för max hastighet i kurva          Bestäm!
v_kurva_ms = gamma * v_max_ms; % Hastighet i kurvan [m/s]
v_accel = 1;        % Sätt en låg hastighet [m/s]
R_kurva = 10;       % Kurvradie [m] (5-15 m)


%% World parameters
g = 9.82;           % Tyngdacceleration [m/s^2]
rho_luft = 1.225;   % Luftens densitet [kg/m^3]


%% Rear axel parameters
L = 1.2; % axel längd
D = 0.1; % axel diameter (tjockare del) Bestäm!
d = 0.6*D; % axel diameter (tunnare del)
R = D/2; % radie (tjockare del)
r = d/2; % radie(tunnare del)

b_1 = 0.15;         % Lagerposition [m] (100-200 mm) Avstånd från axelns ände (y=0)
b_b = 0.08;         % Bromsskiveposition [m] (Ska vara < b_1 enligt tabell)
b_d = 0.25;         % Drevposition [m] (Ska vara b_1 < b_d < L/2)
                    % L/2 = 0.55. Så 0.15 < 0.25 < 0.55. OK.

A = (pi*D^2)/4; % Tvärsnittsarea (tjockare del)
a = (pi*d^2)/4; % Tvärsnittsarea (tunnare del)
K = (pi*D^4)/64; % Vridstyvhetens tvärsnittsfaktor (tjockare del)
k = (pi*d^4)/64; % Vridstyvhetens tvärsnittsfaktor (tunnare del)
I = (pi*D^4)/32; % Areatröghetsmoment x och z (tjockare del)
i = (pi*d^4)/32; % Areatröghetsmoment x och z (tunnare del)

h = 0.001; % tidssteg för iteration
N = L/h; % antalg iterationer

%% Hjulkrafter calc
[FD_accel, Nf_accel, Nb_accel, Mb_accel] = accel_hjulkrafter(df, db, h_luft, hTP, m, Cd, rho_luft, v_accel, a1, r_hjul, g);
[FB_broms, FL_broms, Nf_broms, Nb_broms, Mb_broms] = broms_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms, a2, r_hjul, g);
[V_bi_kurva, V_fi_kurva, V_by_kurva, V_fy_kurva, H_bi_kurva, H_fi_kurva, H_by_kurva, H_fy_kurva, Mb_kurva, FD_kurva] = kurvtagning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_kurva_ms, R_kurva, L, r_hjul, g);
[FD_normal, Nf_normal, Nb_normal, Mb_normal] = körning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms, r_hjul);


%% Lagerkrafter calc
[R_yx_broms, R_ix_broms, R_yz_broms, R_iz_broms, R_iy_broms]=F_lager(L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_max_ms, 2, 0, 0, Nb_broms, Nb_broms, -FB_broms); %Skicka in bromsinput
[R_yx_accel, R_ix_accel, R_yz_accel, R_iz_accel, R_iy_accel]=F_lager(L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, 0, 1, 0, 0, Nb_accel, Nb_accel, FD_accel); %Skicka in accelerationsinput
[R_yx_kurv, R_ix_kurv, R_yz_kurv, R_iz_kurv, R_iy_kurv]=F_lager(L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_kurva_ms, 3, H_bi_kurva, H_by_kurva, V_bi_kurva, V_by_kurva, FD_kurva); %Skicka in kurvtagningsinput

%% Snittstorhet calc
[y_br, Tyx_br, Tyz_br, N_br, Mx_br, My_br] = broms_snittstorheter(L, b_b, b_1, b_d, N, Nb_broms, m, a2, Cd, A_front, rho_luft, v_max_ms, r_hjul, r_broms);
[y_acc, Tyx_acc, Tyz_acc, N_acc, Mx_acc, My_acc] = accel_snittstorheter(L, b_b, b_1, b_d, N, Nb_accel, m, a1, Cd, A_front, rho_luft, v_accel, r_hjul, r_drev);
[y_kurv, Tyx_kurv, Tyz_kurv, N_kurv, Mx_kurv, My_kurv] = kurvtagning_snittstorheter(L, b_b, b_1, b_d, N, V_bi_kurva, V_by_kurva, H_bi_kurva, H_by_kurva);


%% Nominal stress calc
% If 0 is passed -> variable is unused in func
% what to do with y? Svar: Iterera
% To add: M_b1 [is M_b1 == Mb?], M_k, H_i, V_i, V_y
% To calculate: R_ix [se snittstorhet], R_iz [se snittstorhet], R_yz [se snittstorhet], F_k [se komponentkraft-accel]
broms_spanning(y, b_b, b_1, b_d, L, Nb_broms, Mb_broms, 0, 0, 0, 0, 0, 0, 0, 0, 0, a, I, i, K, k); % matcha input parametrar ordning för alla nom stress funktioner
accel_spanning(y, b_b, b_1, b_d, L, Nb_accel, 0, M_k, 0, 0, 0, R_ix, 0, 0, F_k, A, a, I, i, K, k);
kurv_spanning(y, b_b, b_1, b_d, L, 0, 0, 0, H_i, V_i, V_y, 0, R_iz, R_yz, 0, A, a, I, i, 0, 0)
