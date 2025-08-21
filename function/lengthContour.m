function L = lengthContour(C)

    L = 0;
    l = 0;
    
    for i = 1:length(C)-1
    
        if C(1, i) == 0
    
            L = L + l;
            l = 0;
    
        elseif C(1, i + 1) ~= 0
    
            x1 = C(1, i);
            y1 = C(2, i);
    
            x2 = C(1, i + 1);
            y2 = C(2, i + 1);
    
            l = l + sqrt((x1 - x2)^2 + (y1 - y2)^2);
    
        end
    end
    
    L = L + l; % the last iteration
end