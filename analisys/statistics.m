function stats = statistics(data)
% ==========================================================
% STATISTICS.M
% Calcula indicadores estatísticos
% ==========================================================

    stats.mean = mean(data);
    
    stats.std = std(data);
    
    stats.variance = var(data);
    
    stats.maximum = max(data);
    
    stats.minimum = min(data);
    
    stats.range = max(data) - min(data);
    
    stats.rms = rms(data);

end