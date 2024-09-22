function [thpathi] = trajectorygeneration(traj_points, T, Ts)
    n = length(T); % numero de trechos
    cctot = trajectoryplanning(traj_points, T);

    t = 0:Ts:sum(T);
    cumsumT = [0; cumsum(T)];
    thpathi = zeros(length(t), 1);
    for i = 1:n
        trecho_i = (t >= cumsumT(i)) & (t <= cumsumT(i + 1));
        thpathi(trecho_i, 1) = polyval(cctot(i, :), t(trecho_i) - cumsumT(i));
    end
end