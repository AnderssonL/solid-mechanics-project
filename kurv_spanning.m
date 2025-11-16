function [normal, vrid, skjuv] = kurv_spanning(y, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
    
    if 0<=y && y<=b_b
        normal = (H_i/a) + d*(V_i*y)/(i);
        vrid   = 0;
        skjuv  = (4*V_i)/(3*a);

    elseif b_b<y && y<=b_1
        normal = (H_i/a) + d*(V_i*y)/(i);
        vrid   = 0;
        skjuv  = (4*V_i)/(3*a);

    elseif b_1<y && y<=b_d
        normal = (H_i/A) + D*(R_iz*(b_1-y)-V_i-y)/(I);
        vrid   = 0;
        skjuv  = (4*(R_iz+V_i))/(3*A);

    elseif b_d<y && y<=L-b_1
        normal = (H_i/A) + D*(R_iz*(b_1-y)-V_i-y)/(I);
        vrid   = 0;
        skjuv  = (4*(R_iz+V_i))/(3*A);

    elseif L-b_1<y && y<=L-b_b
        normal = (H_i/a) + d*(R_iz*b_1+R_yz*(L-b_1)+V_y*y)/(i);
        vrid   = 0;
        skjuv  = (4*V_i)/(3*a);

    elseif L-b_b<y && y<=L
        normal = (H_i/a) + d*(R_iz*b_1+R_yz*(L-b_1)+V_y*y)/(i);
        vrid   = 0;
        skjuv  = (4*V_i)/(3*a);

    else
        normal = NaN;
        vrid   = NaN;
        skjuv  = NaN;
        disp("Error: y not in range")
    end
end
