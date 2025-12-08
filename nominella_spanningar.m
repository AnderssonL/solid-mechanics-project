function [normal_vec, vridskjuv_vec, tvarskjuv_vec] = nominella_spanningar(y_vec, L, b1, D, d, A, a, K, k, I, ii, Tx_vec, Tz_vec, N_vec, Mx_vec, My_vec, Mz_vec)
    normal_func = @(N, area, Mx, Mz, bojtroghetsmoment, diameter) (N/area) + diameter*sqrt((Mx^2)+(Mz^2))/bojtroghetsmoment;
    vridskjuv_func = @(My, diameter, torsionskonstant) (My*diameter)/(2*torsionskonstant);
    tvarskjuv_func = @(Tx, Tz, area) 4*sqrt((Tx^2)+(Tz^2))/(3*area);

    antal_snitt = numel(N_vec);
    
    normal_vec = zeros(antal_snitt);
    vridskjuv_vec = zeros(antal_snitt);
    tvarskjuv_vec = zeros(antal_snitt);

    for i = 1:antal_snitt
        
        if y_vec(i)<b1 || y_vec(i)>(L+b1)
            bojtroghetsmoment = ii;
            diameter = d;
            area = a;
            torsionskonstant = k;
        else
            bojtroghetsmoment = I;
            diameter = D;
            area = A;
            torsionskonstant = K;
        end

        normal_vec(i) = normal_func(N_vec(i), area, Mx_vec(i), Mz_vec(i), bojtroghetsmoment, diameter);
        vridskjuv_vec(i) = vridskjuv_func(My_vec(i), diameter, torsionskonstant);
        tvarskjuv_vec(i) = tvarskjuv_func(Tx_vec(i), Tz_vec(i), area);
    end
end
