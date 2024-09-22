function [cc] = cubcoef(th0, thdot0, thf, thdotf, T)
    a0 = th0; 
    a1 = thdot0;
    a2 = (3/(T^2))*(thf-th0) - (2*thdot0 + thdotf)/T;
    a3 = -(2/(T^3))*(thf-th0) + (1/(T^2))*(thdotf + thdot0);
    cc = [a3 a2 a1 a0]; % padrao para usar as funcoes polyval e polyder
end