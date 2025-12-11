

b_1 = 0.15;
L=1.2;
drev = b_1:0.01:L/2;
broms = 0.01:0.01:b_1;
%spanningar = zeros(numel(drev),numel(broms));
smallest = 1000000000000000000000;
cords = [0 0];
for x = drev
    disp(x)
    for y = broms
        sp = main_spanning(y,x);
        sp_norm = norm(sp);
        if sp_norm < smallest
            smallest = sp_norm
           
            cords = [x y];
        end
    end
end
cords
smallest