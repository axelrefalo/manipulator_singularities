function w = wedgeProduct(varargin)

    k = nargin;
    v_list = varargin;
    n = numel(v_list{1}); % dimension

    combs = nchoosek(1:n, k);
    w = sym(zeros(size(combs,1), 1));
    
    for i = 1:size(combs,1)

        idx = combs(i,:);
        mat = sym(zeros(k, k));

        for j = 1:k

            mat(j,:) = v_list{j}(idx); 

        end

        % compute the wedge product in the canonical basis of the space of bivectors
        % basis -> oriented plan
        w(i) = det(mat);
    end
end