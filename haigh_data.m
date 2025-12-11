function [n_sakerhet, sigma_u_red, x_vec, y_ideal_vec, y_red_vec, y_yield_vec] = haigh_data(sigma_m, sigma_a, Rm, Rp, sigma_u_tabell, k_yt, k_dim, Kf)
    % HAIGH_DATA Beräknar linjer och säkerhetsfaktor för Haigh-diagram
    %
    % INPUT:
    % sigma_m       - Medelspänning (statisk last, t.ex. drag/tryck) [Pa]
    % sigma_a       - Amplitudspänning (växlande last, t.ex. böjning) [Pa]
    % Rm            - Brottgräns [Pa]
    % Rp            - Sträckgräns [Pa]
    % sigma_u_tabell- Utmattningsgräns från tabell (växlande böjning) [Pa]
    % k_yt          - Ytjämnhetsfaktor (0-1)
    % k_dim         - Dimensionsfaktor (0-1)
    % Kf            - Utmattningsverkan (formfaktor för utmattning, >1)
    %
    % OUTPUT:
    % n_sakerhet    - Beräknad säkerhetsfaktor
    % sigma_u_red   - Den reducerade utmattningsgränsen
    % x_vec         - x-axelns värden (Medelspänning) för plottning
    % y_ideal_vec   - y-värden för ideal Goodman-linje
    % y_red_vec     - y-värden för reducerad Goodman-linje
    % y_yield_vec   - y-värden för sträckgränslinje (Langer)

    %% 1. Beräkna reducerad utmattningsgräns
    % Formel: sigma_u_red = sigma_u_tabell * k_yt * k_dim / Kf
    sigma_u_red = sigma_u_tabell * k_yt * k_dim / Kf;

    %% 2. Beräkna Säkerhetsfaktor
    % Vi kollar både mot utmattning (Goodman) och plasticering (Langer)
    
    % Goodman: 1/n = (sigma_a / sigma_u_red) + (sigma_m / Rm)
    if (sigma_a / sigma_u_red) + (sigma_m / Rm) > 0
        n_goodman = 1 / ( (sigma_a / sigma_u_red) + (sigma_m / Rm) );
    else
        n_goodman = 1000; % Mycket stort om spänningarna är 0
    end
    
    % Langer (mot sträckgräns): n = Rp / (sigma_a + sigma_m)
    if (sigma_a + sigma_m) > 0
        n_langer = Rp / (sigma_a + sigma_m);
    else
        n_langer = 1000;
    end
    
    % Den dimensionerande säkerhetsfaktorn är den lägsta av de två
    n_sakerhet = min(n_goodman, n_langer);

    %% 3. Skapa vektorer för plottning
    % Skapa en x-vektor från 0 till Rm
    steg = 100;
    x_vec = linspace(0, Rm, steg);
    
    % Ideal Goodman-linje (Teoretisk): Från sigma_u_tabell till Rm
    y_ideal_vec = sigma_u_tabell * (1 - x_vec / Rm);
    
    % Reducerad Goodman-linje (Dimensionerande): Från sigma_u_red till Rm
    y_red_vec = sigma_u_red * (1 - x_vec / Rm);
    
    % Sträckgränslinje (Langer): sigma_a + sigma_m = Rp => sigma_a = Rp - sigma_m
    y_yield_vec = Rp - x_vec;
    
    % Snygga till vektorerna (ta bort negativa värden)
    y_ideal_vec(y_ideal_vec < 0) = 0;
    y_red_vec(y_red_vec < 0) = 0;
    y_yield_vec(y_yield_vec < 0) = 0;

end