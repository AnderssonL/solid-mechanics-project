function [normal, vrid, skjuv] = accel_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
    
    if 0<=y && y<=b_b
        normal = d*(N_b*y)/(i);
        vrid   = 0;
        skjuv  = (4*N_b)/(3*a);

    elseif b_b<y && y<=b_1
        normal = d*(N_b*y)/(i);
        vrid   = 0;
        skjuv  = (4*N_b)/(3*a);

    elseif b_1<y && y<=b_d
        normal = D*(N_b*b_1)/(I);
        vrid   = 0;
        skjuv  = (4*R_ix)/(3*A);

    elseif b_d<y && y<=L-b_1
        normal = D*(N_b*b_1)/(I);
        vrid   = D*(M_k)/(2*K);
        skjuv  = (4*(R_ix-F_k))/(3*A);

    elseif L-b_1<y && y<=L-b_b
        normal = d*(N_b*(y+L))/(i);
        vrid   = d*(M_k)/(2*k);
        skjuv  = (4*N_b)/(3*a);

    elseif L-b_b<y && y<=L
        normal = d*(N_b*(y+L))/(i);
        vrid   = d*(M_k)/(2*k);
        skjuv  = (4*N_b)/(3*a);

    else
        normal = NaN;
        vrid   = NaN;
        skjuv  = NaN;
        disp("Error: y not in range")
    end
end
