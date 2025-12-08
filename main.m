% close all;
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


a1 = 6;             % Acceleration i accelerationslastfallet [m/s^2]
a2 = -15;           % Max retardation (värdet är positivt) [m/s^2]
v_max_kmh = 100;    % Maxfart (rakt fram) [km/h] (70-120)
v_max_ms = v_max_kmh / 3.6; % Konvertera maxfart till [m/s]
gamma = 0.8;        % Faktor för max hastighet i kurva          Bestäm!
v_kurva_ms = gamma * v_max_ms; % Hastighet i kurvan [m/s]
v_accel = 1;        % Sätt en låg hastighet [m/s]
R_kurva = 10;       % Kurvradie [m] (5-15 m)

n_s = 2.5;          % Säkerhetsfaktor mot plastisk deformation 
n_u = 2.0;          % Säkerhetsfaktor mot utmattning

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
b_d = 0.6;         % Drevposition [m] (Ska vara b_1 < b_d < L/2)
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

%% Hjulkrafter calc
[FD_accel, Nf_accel, Nb_accel, Mb_accel] = accel_hjulkrafter(df, db, h_luft, hTP, m, Cd, rho_luft, v_accel, a1, r_hjul, g);
[FB_broms, FL_broms, Nf_broms, Nb_broms, Mb_broms] = broms_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms, a2, r_hjul, g);
[V_bi_kurva, V_fi_kurva, V_by_kurva, V_fy_kurva, H_bi_kurva, H_fi_kurva, H_by_kurva, H_fy_kurva, Mb_kurva, FD_kurva, V_i_kurva, V_y_kurva, H_i_kurva, H_y_kurva] = kurvtagning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_kurva_ms, R_kurva, L, r_hjul, g);
[FD_normal, Nf_normal, Nb_normal, Mb_normal] = korning_hjulkrafter(df, db, h_luft, h, m, Cd, rho_luft, v_max_ms, r_hjul, g);

%% Lagerkrafter calc
[R_yx_broms, R_ix_broms, R_yz_broms, R_iz_broms, R_iy_broms, Fk_broms, F_broms_broms]=F_lager(m, L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_max_ms, 2, 0, 0, Nb_broms, Nb_broms, -FB_broms); %Skicka in bromsinput
[R_yx_accel, R_ix_accel, R_yz_accel, R_iz_accel, R_iy_accel, Fk_accel, F_broms_accel]=F_lager(m, L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, 0, 1, 0, 0, Nb_accel/2, Nb_accel/2, FD_accel); %Skicka in accelerationsinput
[R_yx_kurv, R_ix_kurv, R_yz_kurv, R_iz_kurv, R_iy_kurv, Fk_kurv, F_broms_kurv]=F_lager(m, L, b_1, dh, r_broms, r_drev, b_b, b_d, a1, a2, Cd, rho_luft, v_kurva_ms, 3, H_bi_kurva, H_by_kurva, V_bi_kurva, V_by_kurva, FD_kurva); %Skicka in kurvtagningsinput

%% Snittstorhet calc
[y_br, Tyx_br, Tyz_br, N_br, Mx_br, My_br, Mz_br] = anpassad_snittstorheter(2, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, Nb_broms/2, Nb_broms/2, 0,0, R_ix_broms, R_iy_broms, R_iz_broms, R_yx_broms, R_yz_broms, 0, F_broms_broms); 
[y_acc, Tyx_acc, Tyz_acc, N_acc, Mx_acc, My_acc, Mz_acc] = anpassad_snittstorheter(1, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, Nb_accel/2, Nb_accel/2, 0,0, R_ix_accel, R_iy_accel, R_iz_accel, R_yx_accel, R_yz_accel, Fk_accel, 0);
[y_kurv, Tyx_kurv, Tyz_kurv, N_kurv, Mx_kurv, My_kurv, Mz_kurv] = anpassad_snittstorheter(3, L, b_b, b_1, b_d, N, r_hjul, r_drev, r_broms, V_bi_kurva, V_by_kurva, H_bi_kurva, H_by_kurva, R_ix_kurv, R_iy_kurv, R_iz_kurv, R_yx_kurv, R_yz_kurv, Fk_kurv, 0);

%% Nominal stress calc
% If 0 is passed -> variable is unused in func
[normal_br, vrid_br, skjuv_br] = broms_spanning(d, D, N, b_b, b_1, b_d, L, Nb_broms, FB_broms, Mb_broms, 0, 0, 0, 0, 0, 0, 0, 0, 0, a, I, i, K, k); % matcha input parametrar ordning för alla nom stress funktioner.
[normal_acc, vrid_acc, skjuv_acc] = accel_spanning(d, D, N, b_b, b_1, b_d, L, Nb_accel, 0, 0, Mz_acc, 0, 0, 0, R_ix_accel, 0, 0, Fk_accel, A, a, I, i, K, k);
[normal_kurv, vrid_kurv, skjuv_kurv] = kurv_spanning(d, D, N, b_b, b_1, b_d, L, 0, 0, 0, 0, H_bi_kurva, V_bi_kurva, V_by_kurva, 0, R_iz_kurv, R_yz_kurv, 0, A, a, I, i, 0, 0);

effektiv_spanning_br = max(Effektiv_spanning_nominal(normal_br, vrid_br, skjuv_br));
effektiv_spanning_acc = max(Effektiv_spanning_nominal(normal_acc, vrid_acc, skjuv_acc));
effektiv_spanning_kurv = max(Effektiv_spanning_nominal(normal_kurv, vrid_kurv, skjuv_kurv));

disp("Effektivspänning bromsning " + effektiv_spanning_br/1000000 + "MPa");
disp("Effektivspänning acceleration " + effektiv_spanning_acc/1000000 + "MPa");
disp("Effektivspänning kurvtagning " + effektiv_spanning_kurv/1000000 + "MPa");

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
[spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2] = koncentrerad_spanning(normal_kurv, vrid_kurv, K_drev_normal, K_drev_vrid, K_broms_normal, K_broms_vrid, K_lager_normal, K_lager_vrid, L, b_1, b_b, b_d, N);

% beräkna effektivspänning med von mises för 1D balk
[spanneff_drev, spanneff_broms1, spanneff_broms2, spanneff_lager1, spanneff_lager2] = effektiv_spanning(spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2);
spanneff = [spanneff_drev, spanneff_broms1, spanneff_broms2, spanneff_lager1, spanneff_lager2];

% beräkna den sträckgräns som maximala effektivspänning samt säkerhetsfaktorn n_s kräver

[~, idx_max_spanneff] = max(spanneff);
strackgrans = spanneff(idx_max_spanneff) * n_s;
effektiv_spanning_nominal_max = max([effektiv_spanning_br, effektiv_spanning_acc, effektiv_spanning_kurv]);
disp("Den nödvändiga sträckgränsen för att skydda mot lokal plasticering är: " + strackgrans/1000000 + "MPa");
disp("För att undvika plasticering, utan att ta hänsyn till spänningskoncentrationer, krävs en sträckgräns på minst " + n_s * effektiv_spanning_nominal_max/1000000 + " MPa");

%% Plotting
% Skapa figur
figure('Name', 'Snittstorheter Sammanställning', 'NumberTitle', 'off', 'WindowState', 'maximized');

% Layout setup
t = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
xlabel(t, 'Längd [m]', 'FontSize', 12, 'FontWeight', 'bold');
title(t, 'Sammanställning av snittstorheter och spänningar', 'FontSize', 14, 'FontWeight', 'bold');

% Inställningar
lw = 1.5; % Sätt linjetjocklek en gång för att enkelt kunna ändra

% --- RAD 1: BROMSNING ---
% 1. Krafter
nexttile
hold on; grid on; box on;
T_res_br = sqrt(Tyx_br.^2 + Tyz_br.^2); 
plot(y_br, T_res_br/1000, 'LineWidth', lw); 
plot(y_br, N_br/1000, 'LineWidth', lw);
title("Bromsning – Krafter");
ylabel("Kraft [kN]");
legend("T_{res} (Tvärkraft)", "N (Normalkraft)", 'Location', 'best');
xlim([0 L]);

% 2. Moment
nexttile
hold on; grid on; box on;
plot(y_br, Mx_br, 'LineWidth', lw);
plot(y_br, My_br, 'LineWidth', lw);
plot(y_br, Mz_br, 'LineWidth', lw);
title("Bromsning – Moment");
ylabel("Moment [Nm]");
legend("Mx (Böj)", "My (Vrid)", "Mz (Böj)", 'Location', 'best');
xlim([0 L]);

% 3. Spänning
nexttile
hold on; grid on; box on;
plot(y_vector, abs(normal_br./1e6), 'LineWidth', lw)
plot(y_vector, abs(vrid_br./1e6), 'LineWidth', lw)
plot(y_vector, abs(skjuv_br./1e6), 'LineWidth', lw)
title("Bromsning – Spänning")
ylabel("Spänning [MPa]")
legend("Normal", "Vrid", "Skjuv", 'Location', 'best')
xlim([0 L]);

% --- RAD 2: ACCELERATION ---
% 4. Krafter
nexttile
hold on; grid on; box on;
T_res_acc = sqrt(Tyx_acc.^2 + Tyz_acc.^2);
plot(y_acc, T_res_acc/1000, 'LineWidth', lw);
plot(y_acc, N_acc/1000, 'LineWidth', lw);
title("Acceleration – Krafter");
ylabel("Kraft [kN]");
legend("T_{res}", "N", 'Location', 'best');
xlim([0 L]);

% 5. Moment
nexttile
hold on; grid on; box on;
plot(y_acc, Mx_acc, 'LineWidth', lw);
plot(y_acc, My_acc, 'LineWidth', lw);
plot(y_acc, Mz_acc, 'LineWidth', lw);
title("Acceleration – Moment");
ylabel("Moment [Nm]");
legend("Mx", "My", "Mz", 'Location', 'best');
xlim([0 L]);

% 6. Spänning (Flyttad hit för att matcha rad 2)
nexttile
hold on; grid on; box on;
plot(y_vector, abs(normal_acc./1e6), 'LineWidth', lw)
plot(y_vector, abs(vrid_acc./1e6), 'LineWidth', lw)
plot(y_vector, abs(skjuv_acc./1e6), 'LineWidth', lw)
title("Acceleration – Spänning")
ylabel("Spänning [MPa]")
legend("Normal", "Vrid", "Skjuv", 'Location', 'best')
xlim([0 L]);

% --- RAD 3: KURVTAGNING ---
% 7. Krafter
nexttile
hold on; grid on; box on;
T_res_kurv = sqrt(Tyx_kurv.^2 + Tyz_kurv.^2);
plot(y_kurv, T_res_kurv/1000, 'LineWidth', lw);
plot(y_kurv, N_kurv/1000, 'LineWidth', lw);
title("Kurvtagning – Krafter");
ylabel("Kraft [kN]");
legend("T_{res}", "N", 'Location', 'best');
xlim([0 L]);

% 8. Moment
nexttile
hold on; grid on; box on;
plot(y_kurv, Mx_kurv, 'LineWidth', lw);
plot(y_kurv, My_kurv, 'LineWidth', lw);
plot(y_kurv, Mz_kurv, 'LineWidth', lw);
title("Kurvtagning – Moment");
ylabel("Moment [Nm]");
legend("Mx", "My", "Mz", 'Location', 'best');
xlim([0 L]);

% 9. Spänning
nexttile
hold on; grid on; box on;
plot(y_vector, abs(normal_kurv./1e6), 'LineWidth', lw)
plot(y_vector, abs(vrid_kurv./1e6), 'LineWidth', lw)
plot(y_vector, abs(skjuv_kurv./1e6), 'LineWidth', lw)
title("Kurvtagning – Spänning")
ylabel("Spänning [MPa]")
legend("Normal", "Vrid", "Skjuv", 'Location', 'best')
xlim([0 L]);

ax = findobj(gcf,'Type','axes');
linkaxes(ax, 'x');

%% nya plots
%% Plotting - Jämförelse Böjmoment (Figur 3)
% Skapa en ny figur för momentjämförelse
figure('Name', 'Böjmoment Jämförelse', 'NumberTitle', 'off', 'WindowState', 'maximized');
t = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'normal'); % 1 rad, 3 kolumner

% Förbered geometriska linjer och texter
geo_lines = [b_b, b_1, b_d, L-b_1, L-b_b];
geo_labels = {'broms', 'lager', 'drev', 'lager', 'broms'};

% Beräkna resultanter
M_res_acc = sqrt(Mx_acc.^2 + Mz_acc.^2);
M_res_br  = sqrt(Mx_br.^2 + Mz_br.^2);
M_res_kurv= sqrt(Mx_kurv.^2 + Mz_kurv.^2);

% Gemensamma inställningar
lw = 2; % Linjetjocklek
font_sz = 12;

% --- 1. Böjmoment Resultat (Totalen) ---
nexttile
hold on; grid on; box on;
plot(y_acc, M_res_acc, 'r', 'LineWidth', lw);
plot(y_br, M_res_br, 'g', 'LineWidth', lw);
plot(y_kurv, M_res_kurv, 'b', 'LineWidth', lw);

title("Böjmoment resultat", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Böjmoment, [Nm]", 'FontSize', font_sz);
xlabel("x, [m]", 'FontSize', font_sz);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita geometrilinjer
ylim_curr = ylim; % Hämta nuvarande gränser
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'LineWidth', 1, 'Color', [0.4 0.4 0.4]); % Grå streckad
    % Lägg till text roterad 45 grader
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, ...
        'Rotation', 45, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end


% --- 2. Böjmoment Uppifrån (Mz - X-krafter) ---
nexttile
hold on; grid on; box on;
plot(y_acc, Mz_acc, 'r', 'LineWidth', lw);
plot(y_br, Mz_br, 'g', 'LineWidth', lw);
plot(y_kurv, Mz_kurv, 'b', 'LineWidth', lw);

title("Böjmoment Uppifrån", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Böjmoment, [Nm]", 'FontSize', font_sz);
xlabel("x, [m]", 'FontSize', font_sz);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita geometrilinjer
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'LineWidth', 1, 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, ...
        'Rotation', 45, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end


% --- 3. Böjmoment Bakifrån (Mx - Z-krafter) ---
nexttile
hold on; grid on; box on;
plot(y_acc, Mx_acc, 'r', 'LineWidth', lw);
plot(y_br, Mx_br, 'g', 'LineWidth', lw);
plot(y_kurv, Mx_kurv, 'b', 'LineWidth', lw);

title("Böjmoment Bakifrån", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Böjmoment, [Nm]", 'FontSize', font_sz);
xlabel("x, [m]", 'FontSize', font_sz);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita geometrilinjer
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'LineWidth', 1, 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, ...
        'Rotation', 45, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end
%% Plotting - Jämförelse Tvärkrafter (Figur 4)
figure('Name', 'Tvärkraft Jämförelse', 'NumberTitle', 'off', 'WindowState', 'maximized');
t = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'normal');

% Geometri-inställningar
geo_lines = [b_b, b_1, b_d, L-b_1, L-b_b];
geo_labels = {'broms', 'lager', 'drev', 'lager', 'broms'};
lw = 2; 

% Beräkna resultanter
T_res_acc = sqrt(Tyx_acc.^2 + Tyz_acc.^2);
T_res_br  = sqrt(Tyx_br.^2 + Tyz_br.^2);
T_res_kurv= sqrt(Tyx_kurv.^2 + Tyz_kurv.^2);

% --- 1. Tvärkraft Resultat ---
nexttile
hold on; grid on; box on;
plot(y_acc, T_res_acc, 'r', 'LineWidth', lw);
plot(y_br, T_res_br, 'g', 'LineWidth', lw);
plot(y_kurv, T_res_kurv, 'b', 'LineWidth', lw);

title("Tvärkraft resultat", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Tvärkraft, [N]", 'FontSize', 12);
xlabel("x, [m]", 'FontSize', 12);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita linjer & text
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, 'Rotation', 45, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end


% --- 2. Tvärkraft Uppifrån (Tyx) ---
nexttile
hold on; grid on; box on;
plot(y_acc, Tyx_acc, 'r', 'LineWidth', lw);
plot(y_br, Tyx_br, 'g', 'LineWidth', lw);
plot(y_kurv, Tyx_kurv, 'b', 'LineWidth', lw);

title("Tvärkraft Uppifrån", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Tvärkraft, [N]", 'FontSize', 12);
xlabel("x, [m]", 'FontSize', 12);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita linjer & text
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, 'Rotation', 45, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end


% --- 3. Tvärkraft Bakifrån (Tyz) ---
nexttile
hold on; grid on; box on;
plot(y_acc, Tyz_acc, 'r', 'LineWidth', lw);
plot(y_br, Tyz_br, 'g', 'LineWidth', lw);
plot(y_kurv, Tyz_kurv, 'b', 'LineWidth', lw);

title("Tvärkraft Bakifrån", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("Tvärkraft, [N]", 'FontSize', 12);
xlabel("x, [m]", 'FontSize', 12);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');

% Rita linjer & text
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, 'Rotation', 45, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end


%% Plotting - Jämförelse Spänning (Figur 5)
figure('Name', 'Spänning Jämförelse', 'NumberTitle', 'off', 'WindowState', 'maximized');

hold on; grid on; box on;
plot(y_vector, abs(normal_acc), 'r', 'LineWidth', lw);
plot(y_vector, abs(normal_br), 'g', 'LineWidth', lw);
plot(y_vector, abs(normal_kurv), 'b', 'LineWidth', lw);

title("Normalspänning från böjmoment och normalkraft", 'FontSize', 14, 'FontWeight', 'bold');
ylabel("\sigma, [Pa]", 'FontSize', 12);
xlabel("x, [m]", 'FontSize', 12);
legend("Acceleration", "Retardation", "Kurvtagning", 'Location', 'best');
xlim([0 L]);

% Rita linjer & text för spänningsdiagrammet
ylim_curr = ylim;
for k = 1:length(geo_lines)
    xline(geo_lines(k), '--k', 'Color', [0.4 0.4 0.4]);
    text(geo_lines(k), ylim_curr(1), geo_labels{k}, 'Rotation', 45, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 10);
end