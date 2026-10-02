function value = rmse(reference, measured)
% ==========================================================
% RMSE.M
% Calcula o Erro Quadrático Médio
% ==========================================================

    error = measured - reference;
    
    value = sqrt(mean(error.^2));

end