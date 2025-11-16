function [normal, vrid, skjuv] = broms_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
    
    if 0<=y && y<=b_b
        normal = d*(N_b*y)/(i);
        vrid   = 0;
        skjuv  = (4*N_b)/(3*a);

    elseif b_b<y && y<=b_1
        normal = d*(N_b*y)/(i);
        vrid   = d*(M_b1)/(2*k);
        skjuv  = (4*sqrt(F_b^2 + N_b^2))/(3*a);

    elseif b_1<y && y<=b_d
        normal = D*(2*N_b*y+N_b*b_1)/(I);
        vrid   = D*(M_b1)/(2*K);
        skjuv  = 0;

    elseif b_d<y && y<=L-b_1
        normal = D*(2*N_b*y+N_b*b_1)/(I);
        vrid   = D*(M_b1)/(2*K);
        skjuv  = 0;

    elseif L-b_1<y && y<=L-b_b
        normal = d*(N_b*L+N_b*y)/(i);
        vrid   = d*(M_b1)/(2*k);
        skjuv  = (4*sqrt(F_b^2 + N_b^2))/(3*a);

    elseif L-b_b<y && y<=L
        normal = d*(N_b*L+N_b*y)/(i);
        vrid   = 0;
        skjuv  = (4*N_b)/(3*a);

    else
        normal = NaN;
        vrid   = NaN;
        skjuv  = NaN;
        disp("Error: y not in range")
    end
end
