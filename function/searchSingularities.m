function [J] = searchSingularities(urdf_path, DH_table)
    
    robot = loadModel(urdf_path); % load an urdf file for display
    
    vars = symvar(DH_table);

    % Function to create Denavit–Hartenberg (DH) homogeneous transformation
    DH = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                                0,           sin(alpha),             cos(alpha),            d;
                                0,           0,                      0,                     1];
    n = size(DH_table, 1);
    T = cell(n, 1);

    T{1} = eye(4);

    % Denavit–Hartenberg table (classic convention) in [m]
    for i = 2:n

        theta = DH_table(i - 1, 1);
        d     = DH_table(i - 1, 2);
        a     = DH_table(i - 1, 3);
        alpha = DH_table(i - 1, 4);

        T{i} = simplify(T{i - 1} * DH(theta, d, a, alpha));

    end

    % building the Jacobian in the base frame
    Jw = sym(zeros(3,6)); % Rotation part
    Jv = sym(zeros(3,6)); % Translation part
    
    for i = 1:6
    
        w_i = T{i}(1:3, 3); % z-axis of frame i
        q_i = T{i}(1:3, 4); % origin of frame i
    
        Jw(:,i) = w_i;
        Jv(:,i) = cross(q_i, w_i);
    end
    
    % Jacobian in the base frame
    J_base = vpa(simplify([Jw; Jv]));
    
    % Jacobian in another frame
    frame = 4;
    Ti = T{frame};

    R = Ti(1:3,1:3);  % rotation from frame 0 to frame i
    p = Ti(1:3,4);    % translation from frame 0 to frame i
    
    % Skew-symmetric matrix of p
    p_hat = [0,    -p(3),  p(2);
             p(3),  0,    -p(1);
            -p(2),  p(1),  0];
    
    % Inverse adjoint matrix (non complex)
    Ad_inv = [R.',              zeros(3);
             -R.' * p_hat,      R.'];
    
    J = vpa(simplify(Ad_inv * J_base));
    det_J = vpa(det(J));

    fprintf('\n\n Jacobian in frame %d: \n', frame);
    disp(J)
    fprintf('\n\n');

    fprintf('\n\n Determinant: \n');
    disp(det_J)
    fprintf('\n\n');


    % searching for singularities using the wedge product and fmicon
    S = mat2cell(J_base, 6, ones(1,6)); % screws definitions
    
    % store the dependent screws
    dependent_screws = {};

    for k = 2:6

        combos = nchoosek(1:6, k);
        fprintf('\n\ndependencies of %d screws:\n\n', k);

        for i = 1:size(combos,1)
            
            % 1) create the equation
            idx = combos(i,:);
            args = cell(1,k);

            fprintf("solving for columns [%s] -------------------------------------" + ...
                    "\n\n", strjoin(string(idx)));
            
            % copy the current screws into args
            for j = 1:k
                args{j} = S{idx(j)};
            end
            
            wedge_sum_equation = sum(wedgeProduct(args{:}).^2);
            cost_fct = matlabFunction(wedge_sum_equation, 'Vars', {vars});

            % 2) Create the condition
            c = length(idx) - 1;
            wedge_sum_condition = sym(1);

            if c > 1

                % build the subset index based on dependent screws
                sub_idx = {};
                
                for d = length(dependent_screws):-1:1

                    dep = dependent_screws{d};

                    if all(ismember(dep, idx))
                        
                        % check if dep is already in sub_idx
                        already_in = false;
                        for m = 1:length(sub_idx)
                            if all(ismember(dep, sub_idx{m}))
                                already_in = true;
                                break;
                            end
                        end

                        if ~already_in
                            sub_idx{end+1} = dep;
                        end
                    end
                end
                
                for j = 1:length(sub_idx)
                    current_dep = sub_idx{j};
                    args = cell(1, length(current_dep));
                    
                    for l = 1:length(current_dep)
                        args{l} = S{current_dep(l)};
                    end
                    
                    wedge_sum_condition = wedge_sum_condition * sum(wedgeProduct(args{:}).^2);
                end

                fprintf("conditions: ");
                for cond = 1:length(sub_idx)
                    current = sub_idx{cond};
                    fprintf("[%s] ", strjoin(string(current), ' '));
                end
                fprintf("\n");
       
            end

            condition_fct = matlabFunction(wedge_sum_condition, 'Vars', {vars});

            % 3) solve and display

            [qs, cost_value, is_singularity] = optimize(cost_fct, [0, 0, 0, 0, 0, 0], condition_fct);

            if is_singularity

                is_collision = checkCollision(robot, qs, 'SkippedSelfCollisions', 'parent');
                iter = 0;
    
                while(is_collision && iter < 5)
    
                    [qs, cost_value, is_singularity] = optimize(cost_fct, [0, 0, 0, 0, 0, 0], condition_fct);
                    is_collision = checkCollision(robot, qs, 'SkippedSelfCollisions', 'parent');
    
                end

                disp("found: ");
                disp(qs);
                disp("cost: ");
                disp(cost_value);

                label = "singularity " + mat2str(idx);
                displayRobot(robot, label, qs, true, T)

                % saved the dependent axis in a structure called dependent_screws
                dependent_screws{end+1} = sort(idx);

            else

                fprintf("not found\n\n");
                disp("cost: ");
                disp(cost_value);
            
            end
        end
    end
end