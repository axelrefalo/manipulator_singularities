function [qs, cost_value, is_sing] = optimize(cost_fct, x0, condition_fct)

    % lower and upper boundaries
    %     tet1  tet2  tet3  tet4  tet5  tet6
    lb = [0,   -pi,  -pi,  -pi,  -pi,   0];
    ub = [0,    pi,   pi,   pi,   pi,   0];
    
    % condition tolerance 1e-4 is a good one
    epsilon = 0.0001;

    % if the codition if defined
    if nargin < 3 || isempty(condition_fct)
        nonlcon = [];
    else
        nonlcon = @(x) deal(epsilon - condition_fct(x), []);
    end

    % regularize to find solutions closer to x0
    lambda = 1e-12; % 1e-15
    cost_fct_reg = @(x) cost_fct(x) + lambda * norm(x - x0)^2;

    % create a fmincon problem
    problem = createOptimProblem('fmincon', ...
        'objective', cost_fct_reg, ...
        'x0', x0, ...
        'lb', lb, ...
        'ub', ub, ...
        'nonlcon', nonlcon, ...
        'options', optimoptions('fmincon', ...
            'Algorithm', 'interior-point', ...
            'Display', 'off', ...
            'MaxFunctionEvaluations', 1e4, ...
            'OptimalityTolerance', 1e-10));

    % global search object
    gs = GlobalSearch('Display', 'off');

    [qs, fval] = run(gs, problem);

    cost_value = abs(fval);

    if cost_value < 1e-8

        is_sing = true;

    else
        
        is_sing = false;

    end
end