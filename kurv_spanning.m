function [normal, vrid, skjuv] = kurv_spanning(N, b_b, b_1, b_d, L, N_b, M_b1, M_k, H_i, V_i, V_y, R_ix, R_iz, R_yz, F_k, A, a, I, i, K, k)
    % Make outputs vectors
    y_vec = linspace(0, L, N_points); 
    normal = zeros(1, N_points); 
    vrid(i) = zeros(1, N_points);
    skjuv(i) = zeros(1, N_points);

    for i = 1:N_points
        y = y_vec(i);
            
        if 0<=y(i) && y(i)<=b_b
            normal(i) = (H_i/a) + d*(V_i*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*V_i)/(3*a);
    
        elseif b_b<y(i) && y(i)<=b_1
            normal(i) = (H_i/a) + d*(V_i*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*V_i)/(3*a);
    
        elseif b_1<y(i) && y(i)<=b_d
            normal(i) = (H_i/A) + D*(R_iz*(b_1-y(i))-V_i-y(i))/(I);
            vrid(i)   = 0;
            skjuv(i)  = (4*(R_iz+V_i))/(3*A);
    
        elseif b_d<y(i) && y(i)<=L-b_1
            normal(i) = (H_i/A) + D*(R_iz*(b_1-y(i))-V_i-y(i))/(I);
            vrid(i)   = 0;
            skjuv(i)  = (4*(R_iz+V_i))/(3*A);
    
        elseif L-b_1<y(i) && y(i)<=L-b_b
            normal(i) = (H_i/a) + d*(R_iz*b_1+R_yz*(L-b_1)+V_y*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*V_i)/(3*a);
    
        elseif L-b_b<y(i) && y(i)<=L
            normal(i) = (H_i/a) + d*(R_iz*b_1+R_yz*(L-b_1)+V_y*y(i))/(i);
            vrid(i)   = 0;
            skjuv(i)  = (4*V_i)/(3*a);
    
        else
            normal(i) = NaN;
            vrid(i)   = NaN;
            skjuv(i)  = NaN;
            disp("Error: y not in range")
        end
    end
end
