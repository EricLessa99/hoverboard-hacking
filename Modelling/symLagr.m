function [LHS] = symLagr(Lagr, x, t)
    % dL_d(dx_dt)         
    Lqq = deriv(Lagr, diff(x, t));
    % d_dt(dL_d(dx_dt))    
    Lqqt = diff(Lqq, t);
    % dL_dx           
    Lq = deriv(Lagr, x);
    % d_dt(dL_d(dx_dt)) - dL_dx
    LHS = Lqqt - Lq;
end