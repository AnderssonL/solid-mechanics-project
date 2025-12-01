function [spanneff_drev, spanneff_broms1, spanneff_broms2, spanneff_lager1, spanneff_lager2] = effektiv_spanning(spannkonc_drev, spannkonc_broms1, spannkonc_broms2, spannkonc_lager1, spannkonc_lager2)
    % von mises formel
    spanneff_drev   = sqrt(spannkonc_drev(1, 1)^2 + 3*spannkonc_drev(2, 1)^2);

    spanneff_broms1 = sqrt(spannkonc_broms1(1, 1)^2 + 3*spannkonc_broms1(2, 1)^2);
    spanneff_broms2 = sqrt(spannkonc_broms2(1, 1)^2 + 3*spannkonc_broms2(2, 1)^2);

    spanneff_lager1 = sqrt(spannkonc_lager1(1, 1)^2 + 3*spannkonc_lager1(2, 1)^2);
    spanneff_lager2 = sqrt(spannkonc_lager2(1, 1)^2 + 3*spannkonc_lager2(2, 1)^2);
end