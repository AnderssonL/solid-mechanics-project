function [nthn] = formfaktorer_help(d, D, kalradie, r_drev, r_broms)
    names = ["Drev", "Broms", "Lager"]';
    D_div_innerdiam = [2*r_drev/D, 2*r_broms/d, D/d]';
    kalradie_div_innerdiam = [kalradie/D, kalradie/d, kalradie/d]';
    

    disp("Givet tabellen nedan, välj formfaktorer  för normal-spänning samt vrid-spänning");
    table(names, D_div_innerdiam, kalradie_div_innerdiam)

    nthn = NaN;
end
