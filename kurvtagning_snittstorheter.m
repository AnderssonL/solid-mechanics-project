function [y_vec, Tyx, Tyz, N_kraft, Mx, My] = kurvtagning_snittstorheter(L, bb, b1, bd, N_points, Vi, Vy, Hi, Hy)
% KURVTAGNING_SNITTSTORHETER Beräknar snittkrafter för kurvtagning
%
% Inputs:
%   Vi       - Vertikal kraft Inre hjul (N)
%   Vy       - Vertikal kraft Yttre hjul (N)
%   Hi       - Lateral kraft Inre hjul (N)
%   Hy       - Lateral kraft Yttre hjul (N)
%   L, b1... - Geometri

    denominator = 2*b1 - L;
    
    Riz = (Vi * (L - b1) + Vy * (-b1)) / denominator;
    
    Ryz = (-Vi * b1 - Vy * (b1 - L)) / denominator;

    y_vec = linspace(0, L, N_points);
    Tyx = zeros(1, N_points); Tyz = zeros(1, N_points);
    N_kraft = zeros(1, N_points); Mx = zeros(1, N_points); My = zeros(1, N_points);

    for i = 1:N_points
        y = y_vec(i);
        
        if y < b1
            N_kraft(i) = Hi; 
        else
            N_kraft(i) = Hy; 
        end
        
        shear_sum = -Vi;
        moment_sum = -Vi * y;
        
        if y >= b1
            shear_sum = shear_sum - Riz;
            moment_sum = moment_sum - Riz * (y - b1);
        end
        
        if y >= (L - b1)
             shear_sum = shear_sum - Ryz;
             moment_sum = moment_sum - Ryz * (y - (L - b1));
        end
        
        Tyz(i) = shear_sum;
        Mx(i)  = moment_sum;
        
        Tyx(i) = 0;
        My(i)  = 0;
    end
end
