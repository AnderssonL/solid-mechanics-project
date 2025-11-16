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

%% Snittstorhet calc


%% Nominal stress calc
broms_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k); % matcha input parametrar ordning för alla nom stress funktioner
accel_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k);
kurv_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
%% Iteration over axel

% iterera över balk i tidssteg h, beräkna alla spänningar i alla punkter. 
