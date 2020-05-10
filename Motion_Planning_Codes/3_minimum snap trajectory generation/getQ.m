function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];        
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment
        b = n_order;
        for i = 4:b
            for l = 4:b
                Q_k(i+1,l+1) = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)/(i+l-n_order)*(ts(1,1)^(i+l-n_order));
                disp(Q_k);
            end
        end       
        Q = blkdiag(Q, Q_k);
        disp(Q);
    end
end