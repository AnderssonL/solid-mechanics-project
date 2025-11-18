%% Vehicle parameters
% Värdena är tagna från "Typiska värden" i Tabell 2 i projektbeskrivningen.

m = 150;            % Fordonets vikt inkl. förare [kg]
df = 0.8;           % Avstånd framaxel till tyngdpunkt [m] (600-1000 mm)
db = 0.3;           % Avstånd bakaxel till tyngdpunkt [m] (200-400 mm)
h = 0.45;           % Tyngdpunktens vertikala position [m] (30-60 cm)
h1 = 0.2;           % Avstånd tyngdpunkt till luftmotståndets verkningslinje [m] (15-25 cm)
h_luft = h + h1;    % Höjd från marken till luftmotståndets angreppspunkt [m]
A_front = 0.5;      % Fordonets frontarea [m^2]
Cd = 0.3;           % Luftmotståndskoefficient (c i PM)
dh = 0.3;           % Hjuldiameter [m] (240-400 mm)
r_hjul = dh / 2;    % Hjulradie [m]

a1 = 6;             % Acceleration i accelerationslastfallet [m/s^2]
a2 = -15;           % Max retardation (värdet är positivt) [m/s^2]
v_max_kmh = 100;    % Maxfart (rakt fram) [km/h] (70-120)
v_max_ms = v_max_kmh / 3.6; % Konvertera maxfart till [m/s]
v_kurva_ms = gamma * v_max_ms; % Hastighet i kurvan [m/s]
v_accel = 1;        % Sätt en låg hastighet [m/s]
R_kurva = 10;       % Kurvradie [m] (5-15 m)
gamma = 0.8;        % Faktor för max hastighet i kurva (ska egentligen bestämmas)


%% World parameters
g = 9.82;           % Tyngdacceleration [m/s^2]
rho_luft = 1.225;   % Luftens densitet [kg/m^3]


%% Rear axel parameters
L = 1.2; % axel längd
D = 0.1; % axel diameter (tjockare del) Bestäm!
d = 0.6*D; % axel diameter (tunnare del)
R = D/2; % radie (tjockare del)
r = d/2; % radie(tunnare del)

A = (pi*D^2)/4; % Tvärsnittsarea (tjockare del)
a = (pi*d^2)/4; % Tvärsnittsarea (tunnare del)
K = (pi*D^4)/64; % Vridstyvhetens tvärsnittsfaktor (tjockare del)
k = (pi*d^4)/64; % Vridstyvhetens tvärsnittsfaktor (tunnare del)
I = (pi*D^4)/32; % Areatröghetsmoment x och z (tjockare del)
i = (pi*d^4)/32; % Areatröghetsmoment x och z (tunnare del)

h = 0.001; % tidssteg för iteration
N = L/h; % antalg iterationer

%% Hjulkrafter calc
[FD_accel, Nf_accel, Nb_accel, Mb_accel] = accel_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_accel, a1, r_hjul, g);
[FB_broms, FL_broms, Nf_broms, Nb_broms, Mb_broms] = broms_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms, a_broms, r_hjul, g);
[V_bi_kurva, V_fi_kurva, V_by_kurva, V_fy_kurva, H_bi_kurva, H_fi_kurva, H_by_kurva, H_fy_kurva, Mb_kurva] = kurvtagning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_kurva_ms, R_kurva, L, r_hjul, g);
[FD_normal, Nf_normal, Nb_normal, Mb_normal] = körning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms,r_hjul);


%% Lagerkrafter calc


%% Snittstorhet calc


%% Nominal stress calc
broms_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k); % matcha input parametrar ordning för alla nom stress funktioner
accel_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k);
kurv_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
