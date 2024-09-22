function [cctot] = trajectoryplanning(traj_points, T)
    n = length(T); % numero de trechos
    cctot = zeros(n, 4); % inicializa matriz com coeficientes de n polinomios cubicos

    thdot_retas = diff(traj_points)./T; % obtem inclinação de cada reta entre os via points
    thdot = zeros(size(traj_points)); % inicializa velocidade em cada ponto
    for i = 1:n
        if i == n || sign(thdot_retas(i)) ~= sign(thdot_retas(i + 1))
            thdot(i + 1) = 0;
        else
            thdot(i + 1) = (thdot_retas(i) + thdot_retas(i + 1))/2;
        end
        cctot(i, :) = cubcoef(traj_points(i), thdot(i), traj_points(i + 1), thdot(i + 1), T(i));
    end
end