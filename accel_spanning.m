function [normal, vrid, skjuv] = accel_spanning(N, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
    % Make outputs vectors
    y_vec = linspace(0, L, N_points); 
    normal(i) = zeros(1, N_points); 
    vrid(i) = zeros(1, N_points);
    skjuv = zeros(1, N_points);

    for i = 1:N_points
        y = y_vec(i);

        if 0<=y(i) && y(i)<=b_b
            normal(i) = d*(N_b*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif b_b<y(i) && y(i)<=b_1
            normal(i) = d*(N_b*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif b_1<y(i) && y(i)<=b_d
            normal(i) = D*(N_b*b_1)/(I);
            vrid(i)   = 0;
            skjuv(i)  = (4*R_ix)/(3*A);
    
        elseif b_d<y(i) && y(i)<=L-b_1
            normal(i) = D*(N_b*b_1)/(I);
            vrid(i)   = D*(M_k)/(2*K);
            skjuv(i)  = (4*(R_ix-F_k))/(3*A);
    
        elseif L-b_1<y(i) && y(i)<=L-b_b
            normal(i) = d*(N_b*(y(i)+L))/(i);
            vrid(i)   = d*(M_k)/(2*k);
            skjuv(i)  = (4*N_b)/(3*a);
    
        elseif L-b_b<y(i) && y(i)<=L
            normal(i) = d*(N_b*(y(i)+L))/(i);
            vrid(i)   = d*(M_k)/(2*k);
            skjuv(i)  = (4*N_b)/(3*a);
    
        else
            normal(i) = NaN;
            vrid(i)   = NaN;
            skjuv(i)  = NaN;
            disp("Error: y(i) not in range")
        end
    end
end
