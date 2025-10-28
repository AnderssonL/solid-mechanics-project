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

%% Stress functions

% gör minifunktioner för spänningsberäkning

% koordinater: x y z
% snittstorheter = Tx Ty Tz Mx My Mz
% övrigt: sigma (avstånd från balkens neutrallager vid böjning), måste
% lägga till detta^

normalspanning = @(z, Ty, Mx, Mz, A, I) (Ty/A)+sigma*(sqrt((Mx^2)+(Mz^2)))/I; % generellt uttryck

vridskjuvspanning = @(My, K, r) (My*r)/K; % generellt uttryck

tvarskjuvspanning = @(Tx, Tz, A) sqrt((Tx^2)+(Tz^2))/A; % nominellt uttryck

bojskjvspanning = @(x, z, Mx, Mz) ((Mx*z)/I) + ((Mz*x)/I); % generellt uttryck



%% Nominal stress functions




%% Iteration over axel

% iterera över balk i tidssteg h, beräkna alla spänningar i alla punkter. 
