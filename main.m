close all;
%% Vehicle parameters
% Värdena är tagna från "Typiska värden" i Tabell 2 i projektbeskrivningen.
m = 150;            % Fordonets vikt inkl. förare [kg]
df = 0.8;           % Avstånd framaxel till tyngdpunkt [m] (600-1000 mm)
db = 0.3;           % Avstånd bakaxel till ty   ngdpunkt [m] (200-400 mm)
hTP = 0.45;           % Tyngdpunktens vertikala position [m] (30-60 cm)
hFL = 0.2;           % Avstånd tyngdpunkt till luftmotståndets verkningslinje [m] (15-25 cm)
h_luft = hTP+ hFL;    % Höjd från marken till luftmotståndets angreppspunkt [m]
A_front = 0.5;      % Fordonets frontarea [m^2]
Cd = 0.3;           % Luftmotståndskoefficient (c i PM)
dh = 0.3;           % Hjuldiameter [m] (240-400 mm)
r_hjul = dh / 2;    % Hjulradie [m]
r_drev = 0.3*dh;      % Drevets radie [m] 
r_broms = 0.2*dh;     % Bromsskivans radie [m]
kalradie = 4e-3;   % Kälradie [m]

g = 9.82;           % Tyngdacceleration [m/s^2]
rho_luft = 1.225;   % Luftens densitet [kg/m^3]

%% Rear axel parameters
L = 1.2; % axel längd
D = 0.056; % axel diameter (tjockare del) Bestäm!
d = 0.6*D; % axel diameter (tunnare del)
R = D/2; % radie (tjockare del)
r = d/2; % radie(tunnare del)

b_1 = 0.15;         % Lagerposition [m] (100-200 mm) Avstånd från axelns ände (y=0)
b_b = 0.11;         % Bromsskiveposition [m] (Ska vara < b_1 enligt tabell)
b_d = 0.15;         % Drevposition [m] (Ska vara b_1 < b_d < L/2)
                    % L/2 = 0.55. Så 0.15 < 0.25 < 0.55. OK.

A = (pi*D^2)/4; % Tvärsnittsarea (tjockare del)
a = (pi*d^2)/4; % Tvärsnittsarea (tunnare del)
K = (pi*D^4)/64; % Vridstyvhetens tvärsnittsfaktor (tjockare del)
k = (pi*d^4)/64; % Vridstyvhetens tvärsnittsfaktor (tunnare del)
I = (pi*D^4)/32; % Areatröghetsmoment x och z (tjockare del)
i = (pi*d^4)/32; % Areatröghetsmoment x och z (tunnare del)

h = 0.001; % tidssteg för iteration
N = L/h; % antalg iterationer
y_vector = linspace(0, L, N);

%% Acceleration and force parameters
a1 = 6;             % Acceleration i accelerationslastfallet [m/s^2]
a2 = 15;           % Max retardation (värdet är positivt) [m/s^2]
v_max_ms = 72/3.6;    % Maxfart (rakt fram) [km/h] (70-120)
R_kurva = 15;       % Kurvradie [m] (5-15 m)
gamma = (1/v_max_ms)*sqrt((R_kurva*g*L)/(2*hTP));        % Faktor för max hastighet i kurva          Bestäm!
v_kurva_ms = gamma * v_max_ms; % Hastighet i kurvan [m/s]
v_accel = 1;        % Sätt en låg hastighet [m/s]

n_s = 2.5;          % Säkerhetsfaktor mot plastisk deformation 
n_u = 2.0;          % Säkerhetsfaktor mot utmattning

%% Hjulkrafter calc
[FD_accel, Nf_accel, Nb_accel, Mb_accel] = accel_hjulkrafter(df, db, hFL, hTP, m, Cd, rho_luft, v_accel, a1, r_hjul, g);
[FB_broms, FL_broms, Nf_broms, Nb_broms, Mb_broms] = broms_hjulkrafter(df, db, hFL, hTP, m, Cd, rho_luft, v_max_ms, a2, r_hjul, g);
[V_bi_kurva, V_fi_kurva, V_by_kurva, V_fy_kurva, H_bi_kurva, H_fi_kurva, H_by_kurva, H_fy_kurva, Mb_kurva, FD_kurva, V_i_kurva, V_y_kurva, H_i_kurva, H_y_kurva] = kurvtagning_hjulkrafter(df, db, hFL, hTP, m, Cd, rho_luft, v_kurva_ms, R_kurva, L, r_hjul, g);

%% Lagerkrafter calc
[R_yx_broms, R_ix_broms, R_yz_broms, R_iz_broms, R_iy_broms, Fk_broms, F_broms_broms]=F_lager(m, L, b_1, r_hjul, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_max_ms, 2, 0, 0, Nb_broms/2, Nb_broms/2, -FB_broms); %Skicka in bromsinput
[R_yx_accel, R_ix_accel, R_yz_accel, R_iz_accel, R_iy_accel, Fk_accel, F_broms_accel]=F_lager(m, L, b_1, r_hjul, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_accel, 1, 0, 0, Nb_accel/2, Nb_accel/2, FD_accel); %Skicka in accelerationsinput
[R_yx_kurv, R_ix_kurv, R_yz_kurv, R_iz_kurv, R_iy_kurv, Fk_kurv, F_broms_kurv]=F_lager(m, L, b_1, r_hjul, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_kurva_ms, 3, H_bi_kurva, H_by_kurva, V_bi_kurva, V_by_kurva, FD_kurva); %Skicka in kurvtagningsinput

%% Snittstorhet calc
[y_br, Tyx_br, Tyz_br, N_br, Mx_br, My_br, Mz_br] = anpassad_snittstorheter(2, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, Nb_broms/2, 0, R_ix_broms, R_iy_broms, R_iz_broms, R_yx_broms, R_yz_broms, 0, F_broms_broms); 
[y_acc, Tyx_acc, Tyz_acc, N_acc, Mx_acc, My_acc, Mz_acc] = anpassad_snittstorheter(1, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, Nb_accel/2, 0, R_ix_accel, R_iy_accel, R_iz_accel, R_yx_accel, R_yz_accel, Fk_accel, 0);
[y_kurv, Tyx_kurv, Tyz_kurv, N_kurv, Mx_kurv, My_kurv, Mz_kurv] = anpassad_snittstorheter(3, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, V_bi_kurva, H_bi_kurva, R_ix_kurv, R_iy_kurv, R_iz_kurv, R_yx_kurv, R_yz_kurv, Fk_kurv, 0);

%% Nominal stress calc
[normal_br, vridskjuv_br, tvarskjuv_br] = nominella_spanningar(y_br, L, b_1, D, d, A, a, K, k, I, i, Tyx_br, Tyz_br, N_br, Mx_br, My_br, Mz_br);
[normal_acc, vridskjuv_acc, tvarskjuv_acc] = nominella_spanningar(y_acc, L, b_1, D, d, A, a, K, k, I, i, Tyx_acc, Tyz_acc, N_acc, Mx_acc, My_acc, Mz_acc);
[normal_kurv, vridskjuv_kurv, tvarskjuv_kurv] = nominella_spanningar(y_kurv, L, b_1, D, d, A, a, K, k, I, i, Tyx_kurv, Tyz_kurv, N_kurv, Mx_kurv, My_kurv, Mz_kurv);

effektiv_spanning_br = max(Effektiv_spanning_nominal(normal_br, vridskjuv_br, tvarskjuv_br));
effektiv_spanning_acc = max(Effektiv_spanning_nominal(normal_acc, vridskjuv_acc, tvarskjuv_acc));
effektiv_spanning_kurv = max(Effektiv_spanning_nominal(normal_kurv, vridskjuv_kurv, tvarskjuv_kurv));
disp("Effektivspänning broms " + effektiv_spanning_br/1000000);
disp("Effektivspänning acceleration " + effektiv_spanning_acc/1000000);
disp("Effektivspänning kurvtagning " + effektiv_spanning_kurv/1000000);

effektiv_spanning_br = Effektiv_spanning_nominal(normal_br, vridskjuv_br, tvarskjuv_br);
effektiv_spanning_acc = Effektiv_spanning_nominal(normal_acc, vridskjuv_acc, tvarskjuv_acc);
effektiv_spanning_kurv = Effektiv_spanning_nominal(normal_kurv, vridskjuv_kurv, tvarskjuv_kurv);

%% Lokala spänningskonc calc (för kurvtagning)
% Hjälpfunktion för att bestämma formfaktorer (se tabell 32.4 i formelsamlingen)
formfaktorer_help(d, D, kalradie, r_drev, r_broms);

% givet tabellen:
K_drev_normal  = 2.70;
K_drev_vrid    = 1.85;

K_broms_normal = 2.35;
K_broms_vrid   = 1.65;

K_lager_normal = 2.20;
K_lager_vrid   = 1.60;

% beräkna lokala spänningskonc för drev (1 st), broms (2 st), lager (2 st)
% ex: col1 = broms1, col2 = broms2, row1 = normalspänning, row2 = vridspänning
[spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2] = koncentrerad_spanning(normal_br, vridskjuv_br, K_drev_normal, K_drev_vrid, K_broms_normal, K_broms_vrid, K_lager_normal, K_lager_vrid, L, b_1, b_b, b_d, N);
spannkonc = [spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2];
% beräkna effektivspänning med von mises för 1D balk
[spanneff_drev, spanneff_broms1, spanneff_broms2, spanneff_lager1, spanneff_lager2] = effektiv_spanning(spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2);
spanneff = [spanneff_drev, spanneff_broms1, spanneff_broms2, spanneff_lager1, spanneff_lager2];

% beräkna den sträckgräns som maximala effektivspänning samt säkerhetsfaktorn n_s kräver

[~, idx_max_spanneff] = max(spanneff);
strackgrans = spanneff(idx_max_spanneff) * n_s;
effektiv_spanning_nominal_max = max([effektiv_spanning_br, effektiv_spanning_acc, effektiv_spanning_kurv]);
disp("För att undvika plasticering, utan att ta hänsyn till spänningskoncentrationer, krävs en sträckgräns på minst " + n_s * effektiv_spanning_nominal_max/1000000 + " MPa");
disp("Den nödvändiga sträckgränsen för att skydda mot lokal plasticering är: " + strackgrans/1000000 + "MPa");


%% Utmattningsparametrar och Haigh-diagram

% Materialdata (Givet i din kod/tabell)
sigma_ub = 460e6;   % Utmattningsgräns (växlande böjning)
Rm = 860e6;         % Brottgräns
Rp02 = 550e6;       % Sträckgräns

% --- 1. Bestäm formfaktor Kt och Utmattningsverkan Kf ---
% Vi väljer det värsta fallet av formfaktorerna vi har (t.ex. broms eller drev).
% Här tar vi maximum av de normal-Kt vi har definierat tidigare.
Kt_utm = max([K_drev_normal, K_broms_normal, K_lager_normal]); 

q = 0.9; % Känslighetsfaktor (Givet)
Kf_utm = 1 + q * (Kt_utm - 1); % Beräkning av utmattningsverkan

% --- 2. Bestäm övriga faktorer ---
% Dimensionsfaktor (k_dim): För axlar d=40-50mm är ca 0.8 rimligt. 
% (Du kan göra detta mer exakt beroende på diameter d eller D).
if D < 0.05
    Kd_utm = 0.85;
else
    Kd_utm = 0.81; % Ungefärligt värde för 50mm
end

% Ytfaktor (k_yt): Beroende på Rm och bearbetning (Svarvad).
% För Rm=860 och svarvad yta ligger faktorn ofta runt 0.8 - 0.85.
Ky_utm = 0.82; 

% --- 3. Hämta spänningar från värsta lastfallet (Kurvtagning) ---
% Vi måste hitta var på axeln spänningen är högst för att kolla utmattning där.
% Vi använder effektivspänningen (von Mises) som amplitudspänning för att vara säkra,
% även om Haigh strikt sett är för normalspänning.
[max_eff_stress, idx_max] = max(effektiv_spanning_br);

% Sigma_a (Amplitud): Det som böjer axeln när den snurrar.
% Vi använder von mises-spänningen (eller bara böjspänningen) som amplitud.
sigma_a_input = max_eff_stress; 

% Sigma_m (Medel): Statisk last (Drag/Tryck från N).
% I kurvtagning finns en axiell last N_kurv.
if y_kurv(idx_max) < b_1 || y_kurv(idx_max) > L-b_1
    area_vid_snitt = a; % Tunnare del
else
    area_vid_snitt = A; % Tjockare del
end
sigma_m_input = abs(N_kurv(idx_max)) / area_vid_snitt;

% --- 4. Kör Haigh-funktionen ---
[n_haigh, sigma_u_red, h_x, h_y_ideal, h_y_red, h_y_yield] = haigh_data(sigma_m_input, sigma_a_input, Rm, Rp02, sigma_ub, Ky_utm, Kd_utm, Kf_utm);

disp("------------------------------------------------");
disp("HAIGH-RESULTAT (Kurvtagning - Dimensionerande snitt)");
disp("Vald Kt (max): " + Kt_utm);
disp("Beräknad Kf: " + Kf_utm);
disp("Reducerad utmattningsgräns: " + sigma_u_red/1e6 + " MPa");
disp("Arbetspunkt: Sigma_a=" + sigma_a_input/1e6 + " MPa, Sigma_m=" + sigma_m_input/1e6 + " MPa");
disp("Säkerhetsfaktor mot utmattning: " + n_haigh);
disp("------------------------------------------------");


%% PLOTTNING

lw = 1.5;
% ===========================
%  Figur 1 — Accel Krafter
% ===========================
figure(1); clf;
hold on; grid on; box on;
plot(y_acc, Tyx_acc/1000, 'r', 'LineWidth', lw);
plot(y_acc, Tyz_acc/1000, 'b', 'LineWidth', lw);
plot(y_acc, N_acc/1000,   'k', 'LineWidth', lw);
title('Accel: Krafter'); ylabel('Kraft [kN]'); xlabel("y [m]"); xlim([0 L]);
legend('T_x', 'T_z', 'N', 'Location', 'best');


% ===========================
%  Figur 2 — Accel Moment
% ===========================
figure(2); clf;
hold on; grid on; box on;
plot(y_acc, Mx_acc, 'r', 'LineWidth', lw);
plot(y_acc, My_acc, 'g', 'LineWidth', lw);
plot(y_acc, Mz_acc, 'b', 'LineWidth', lw);
title('Accel: Moment'); ylabel('Moment [Nm]'); xlabel("y [m]"); xlim([0 L]);
legend('M_x','M_y','M_z','Location','best');


% ===========================
%  Figur 3 — Accel Spänningar
% ===========================
figure(3); clf;
hold on; grid on; box on;
plot(y_vector, abs(normal_acc)/1e6, 'r', 'LineWidth', lw);
plot(y_vector, abs(vridskjuv_acc)/1e6, 'g', 'LineWidth', lw);
plot(y_vector, abs(tvarskjuv_acc)/1e6, 'b', 'LineWidth', lw);
title('Accel: Spänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma', '\tau_v', '\tau_s','Location','best');


% ===========================
%  Figur 4 — Broms Krafter
% ===========================
figure(4); clf;
hold on; grid on; box on;
plot(y_br, Tyx_br/1000, 'r', 'LineWidth', lw);
plot(y_br, Tyz_br/1000, 'b', 'LineWidth', lw);
plot(y_br, N_br/1000,   'k', 'LineWidth', lw);
title('Broms: Krafter'); ylabel('Kraft [kN]'); xlabel("y [m]"); xlim([0 L]);
legend('T_x', 'T_z', 'N', 'Location', 'best');

% ===========================
%  Figur 5 — Broms Moment
% ===========================
figure(5); clf;
hold on; grid on; box on;
plot(y_br, Mx_br, 'r', 'LineWidth', lw);
plot(y_br, My_br, 'g', 'LineWidth', lw);
plot(y_br, Mz_br, 'b', 'LineWidth', lw);
title('Broms: Moment'); ylabel('Moment [Nm]'); xlabel("y [m]"); xlim([0 L]);
legend('M_x','M_y','M_z','Location','best');

% ===========================
%  Figur 6 — Broms Spänningar
% ===========================
figure(6); clf;
hold on; grid on; box on;
plot(y_vector, abs(normal_br)/1e6, 'r', 'LineWidth', lw);
plot(y_vector, abs(vridskjuv_br)/1e6, 'g', 'LineWidth', lw);
plot(y_vector, abs(tvarskjuv_br)/1e6, 'b', 'LineWidth', lw);
title('Broms: Spänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma', '\tau_v', '\tau_s','Location','best');


% ===========================
%  Figur 7 — Kurv Krafter
% ===========================
figure(7); clf;
hold on; grid on; box on;
plot(y_kurv, Tyx_kurv/1000, 'r', 'LineWidth', lw);
plot(y_kurv, Tyz_kurv/1000, 'b', 'LineWidth', lw);
plot(y_kurv, N_kurv/1000,   'k', 'LineWidth', lw);
title('Kurv: Krafter'); ylabel('Kraft [kN]'); xlabel("y [m]"); xlim([0 L]);
legend('T_x', 'T_z', 'N', 'Location', 'best');


% ===========================
%  Figur 8 — Kurv Moment
% ===========================
figure(8); clf;
hold on; grid on; box on;
plot(y_kurv, Mx_kurv, 'r', 'LineWidth', lw);
plot(y_kurv, My_kurv, 'g', 'LineWidth', lw);
plot(y_kurv, Mz_kurv, 'b', 'LineWidth', lw);
title('Kurv: Moment'); ylabel('Moment [Nm]'); xlabel("y [m]"); xlim([0 L]);

% ===========================
%  Figur 9 — Kurv Spänningar
% ===========================
figure(9); clf;
hold on; grid on; box on;
plot(y_vector, abs(normal_kurv)/1e6, 'r', 'LineWidth', lw);
plot(y_vector, abs(vridskjuv_kurv)/1e6, 'g', 'LineWidth', lw);
plot(y_vector, abs(tvarskjuv_kurv)/1e6, 'b', 'LineWidth', lw);
title('Kurv: Spänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma', '\tau_v', '\tau_s','Location','best');



% ===========================
%  Figur 10 — Broms Effektivspänning
% ===========================
figure(10); clf;
hold on; grid on; box on;
plot(y_vector, effektiv_spanning_br/1e6, 'r', 'LineWidth', lw);
title('Broms: Effektivspänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma_{vM}','Location','best');

% ===========================
%  Figur 11 — Acceleration Effektivspänning
% ===========================
figure(11); clf;
hold on; grid on; box on;
plot(y_vector, effektiv_spanning_acc/1e6, 'r', 'LineWidth', lw);
title('Acceleration: Effektivspänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma_{vM}','Location','best');

% ===========================
%  Figur 12 — Kurvtagning Effektivspänning
% ===========================
figure(12); clf;
hold on; grid on; box on;
plot(y_vector, effektiv_spanning_kurv/1e6, 'r', 'LineWidth', lw);
title('Kurvtagning: Effektivspänning'); ylabel('Spänning [MPa]'); xlabel("y [m]"); xlim([0 L]);
legend('\sigma_{vM}','Location','best');

% ===========================
%  Figur 13 — Sammaställd Effektivspänning
% ===========================
figure(13); clf;
hold on; grid on; box on;
plot(y_vector, effektiv_spanning_br/1e6, 'r', 'LineWidth', lw);
plot(y_vector, effektiv_spanning_acc/1e6, 'b', 'LineWidth', lw);
plot(y_vector, effektiv_spanning_kurv/1e6, 'g', 'LineWidth', lw);
title('Effektivspänning'); 
ylabel('Spänning [MPa]'); 
xlabel("y [m]"); xlim([0 L]);
legend('\sigma_{vM, Broms}', '\sigma_{vM, Acceleration}', '\sigma_{vM, Kurvtagning}', 'Location','best');

% ===========================
%  Figur 14 — Haigh-Diagram
% ===========================
figure(14); clf;
hold on; grid on; box on;
fill([h_x fliplr(h_x)]/1e6, [h_y_red fliplr(zeros(size(h_y_red)))]/1e6, [0.9 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3); % Fyll säkert område
plot(h_x/1e6, h_y_ideal/1e6, 'k--', 'LineWidth', 1.0);
plot(h_x/1e6, h_y_red/1e6, 'r', 'LineWidth', 2.0);
plot(h_x/1e6, h_y_yield/1e6, 'b-.', 'LineWidth', 1.5);
plot(sigma_m_input/1e6, sigma_a_input/1e6, 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

xlabel('Medelspänning \sigma_m [MPa]');
ylabel('Amplitudspänning \sigma_a [MPa]');
title(['Haigh-diagram (Reducerat), n_{utm} = ' num2str(n_haigh, '%.2f')]);
legend('Säkert område', 'Ideal gräns (Goodman)', 'Reducerad gräns (Design)', 'Sträckgräns (Langer)', 'Arbetspunkt', 'Location', 'NorthEast');
axis([0 Rm/1e6 0 Rm/1e6]);