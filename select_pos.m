close all

b_1 = 0.15;
L=1.2;
drev = b_1:0.01:L/2;
broms = 0.01:0.01:b_1;
%spanningar = zeros(numel(drev),numel(broms));
smallest = 1000000000000000000000;
cords = [0 0];

XY = drev'*broms;
SP = zeros(size(XY));
size(SP)
size(drev)
size(broms)



for x = drev
    disp(x);
    
    for y = broms
        
        sp = main_spanning(y,x);
        sp_norm = norm(sp);
        
        SP(round(x*100-14),round(y*100)) = sp_norm;
        if sp_norm < smallest
            smallest = sp_norm
            
            cords = [x y];
        end
    end
end
cords
smallest

surf(broms, drev,SP)
xlabel('Broms position')
ylabel('Drev position')
zlabel('Spänning')