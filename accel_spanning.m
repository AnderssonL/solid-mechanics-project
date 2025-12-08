function [normal, vrid, skjuv] = accel_spanning(d, D, N_points, b_b, b_1, b_d, L, N_b, F_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, ii, K, k)
    % Make outputs vectors
    y_vec = linspace(0, L, N_points); 
    normal = zeros(1, N_points); 
    vrid = zeros(1, N_points);
    skjuv = zeros(1, N_points);


    for i = 1:N_points
        y = y_vec(i);

        if 0<=y && y<=b_b
            normal(i) = d*(N_b*y)/(ii);
            vrid(i)   = 0;
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif b_b<y && y<=b_1
            normal(i) = d*(N_b*y)/(ii);
            vrid(i)   = 0;
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif b_1<y && y<=b_d
            normal(i) = D*(N_b*b_1)/(I);
            vrid(i)   = 0;
            skjuv(i)  = (4*R_ix)/(3*A);
    
        elseif b_d<y && y<=L-b_1
            normal(i) = D*(N_b*b_1)/(I);
            vrid(i)   = D*(M_k(i))/(2*K);
            skjuv(i)  = (4*(R_ix-F_k))/(3*A);
    
        elseif L-b_1<y && y<=L-b_b
            normal(i) = d*(N_b*(y+L))/(ii);
            vrid(i)   = d*(M_k(i))/(2*k); 
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif L-b_b<y && y<=L
            normal(i) = d*(N_b*(y+L))/(ii);
            vrid(i)   = d*(M_k(i))/(2*k);
            skjuv(i)  = (4*N_b)/(3*a);
    
        else
            normal(i) = NaN;
            vrid(i)   = NaN;
            skjuv(i)  = NaN;
            disp("Error: y not in range")
        end
    end
end
