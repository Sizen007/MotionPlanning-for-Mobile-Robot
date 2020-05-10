function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    %
    %
    %
    %
    Aeq_start(1,1) = 1;
    Aeq_start(2,2) = 1;
    Aeq_start(3,3) = 2;
    Aeq_start(4,4) = 6;

    beq_start(1,1) = waypoints(1);
    beq_start(2,1) = 0;
    beq_start(3,1) = 0;
    beq_start(4,1) = 0;

    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %
    beq_end = [waypoints(end); 0; 0; 0];
    end_begin = (n_seg-1)*(n_order+1)+1;
    for k = 1:4 %k is the derivate index
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
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    %
    for i=1:n_seg-1
        for j=1:n_order+1
            Aeq_wp(i, (i-1)*(n_order+1)+j) = ts(i)^(j-1);
        end
    end
    beq_wp = waypoints(2:end-1);
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %
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
    
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
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
        disp(Aeq_con_v);
    end

    
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
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
    
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    k = 3;
    for l = 1:n_seg-1
        n = l-1;
        for i = 1:8
            m = i-1;
            if m>=k
                Aeq_con_j(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            else
                Aeq_con_j(l,i+8*n) = 0;
            end
        end
        Aeq_con_j(l,12+8*n) = -6;
        disp(Aeq_con_j);
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end