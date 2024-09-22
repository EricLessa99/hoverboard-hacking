function [newExpression] = subsExpression(oldExpression, old_vars, new_vars)
    n_exp = length(oldExpression);
    n_vars = length(new_vars);
    newExpression = oldExpression;
    for idx_exp = 1:n_exp
        newExpression(idx_exp) = oldExpression(idx_exp);
        for idx_vars = 1:n_vars
            newExpression(idx_exp) = subs(newExpression(idx_exp), old_vars(idx_vars), new_vars(idx_vars));
        end
    end
end