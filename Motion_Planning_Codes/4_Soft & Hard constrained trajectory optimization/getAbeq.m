function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond,M)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1);
    Aeq_start(1,1) = 1;
    Aeq_start(2,2) = 1;
    Aeq_start(3,3) = 2;

    beq_start(1,1) = start_cond(1,1);
    beq_start(2,1) = 0;
    beq_start(3,1) = 0;
    
    Aeq_start = Aeq_start*M;
    %disp(Aeq_start);
    %disp(beq_start);
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    beq_end = zeros(3, 1);
    beq_end(1,1) = end_cond(1,1);
    end_begin = (n_seg-1)*(n_order+1)+1;
    for k = 1:3 %k is the derivate index
        m = k-1;
        for i = 1:8
            n = i-1;
            if n>=m
                Aeq_end(k,end_begin+n) = factorial(n)/factorial(n-m)*ts(1,1)^(n-m);
            else
                Aeq_end(k,end_begin+n) = 0;
            end
            %disp(Aeq_end);
        end
    end
    Aeq_end = Aeq_end*M;
    %disp(Aeq_end);
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    k = 0;
    for l = 1: n_seg-1
        n = l-1;
        for i = 1:9
            m = i-1;
            if i == 9
                Aeq_con_p(l,i+8*n) = -1;
            else
                Aeq_con_p(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            end
            %disp(Aeq_con_p);
        end
    end
    Aeq_con_p = Aeq_con_p*M;
    %disp(Aeq_con_p);
    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    
    k = 1;
    for l = 1:n_seg-1
        n = l-1;
        for i = 1:8
            m = i-1;
            if m>=k
                Aeq_con_v(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            else
                Aeq_con_v(l,i+8*n) = 0;
            end
        end
        Aeq_con_v(l,10+8*n) = -1;
        %disp(Aeq_con_v);
    end
    Aeq_con_v = Aeq_con_v*M;
    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    k = 2;
    for l = 1:n_seg-1
        n = l-1;
        for i = 1:8
            m = i-1;
            if m>=k
                Aeq_con_a(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            else
                Aeq_con_a(l,i+8*n) = 0;
            end
        end
        Aeq_con_a(l,11+8*n) = -2;
        %disp(Aeq_con_a);
    end
    Aeq_con_a = Aeq_con_a*M;
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end