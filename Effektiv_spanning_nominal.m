%sigma_x = @(x) 160+x; %Alla värden i MPa
%sigma_y =  @(x) 100; 
%sigma_z = @(x) 0;
%tao_xy =  @(x) 80;
%tao_xz = @(x) 0;
%tao_yz =  @(x) 0;
%sigma_von_mises = @(x) sqrt(sigma_x(x).^2 + sigma_y(x).^2 + sigma_z(x).^2 - sigma_x(x).*sigma_y(x) - sigma_x(x).*sigma_z(x) - sigma_y(x).*sigma_z(x) + 3*(tao_xy(x).^2 + tao_xz(x).^2 + tao_yz(x).^2));
%x = 0:100;
%disp(sigma_von_mises(x));
%plot(x,sigma_von_mises(x));
%%axis([0 10 0 300])
%
function effektiv_spanning = Effective_spanning_nominal(normal, vrid, skjuv)

effektiv_spanning = sqrt(normal.^2 + 3*(vrid+skjuv).^2);% + 3*skjuv.^2);

end