function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max,M)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [];
    Aieq_p2 =[];
    Aieq_p3 = [];
    bieq_p = [];
    
    for k = 1:n_seg%%small than
        Aieq_p_k =   [1,0,0,0,0,0,0,0;
                      0,1,0,0,0,0,0,0;
                      0,0,1,0,0,0,0,0;
                      0,0,0,1,0,0,0,0;
                      0,0,0,0,1,0,0,0;
                      0,0,0,0,0,1,0,0;
                      0,0,0,0,0,0,1,0;
                      0,0,0,0,0,0,0,1;];                    
        Aieq_p = blkdiag(Aieq_p, Aieq_p_k);      
    end
    for k = 1:n_seg%%more than
        Aieq_p_k =   [-1,0,0,0,0,0,0,0;
                      0,-1,0,0,0,0,0,0;
                      0,0,-1,0,0,0,0,0;
                      0,0,0,-1,0,0,0,0;
                      0,0,0,0,-1,0,0,0;
                      0,0,0,0,0,-1,0,0;
                      0,0,0,0,0,0,-1,0;];                   
        Aieq_p2 = blkdiag(Aieq_p2, Aieq_p_k);      
    end    
    
    Aieq_p = [Aieq_p;Aieq_p2];
    
    for k = 1:n_seg%%end
        Aieq_p_k =   [0,0,0,0,0,0,0,-1];                 
        Aieq_p3 = blkdiag(Aieq_p3, Aieq_p_k);      
    end    
    
    Aieq_p = [Aieq_p;Aieq_p3];
    %
    for i = 1:n_seg
        n = i-1;
        for j = 1: n_order+1
            bieq_p_1(j+n*8,1) = corridor_range(i,2);
        end       
    end
  
    for i = 1:n_seg
        n = i-1;
        for j = 1: n_order
            bieq_p_2(j+n*7,1) = -corridor_range(i,1);           
        end       
    end
    bieq_p = [bieq_p_1;bieq_p_2];
    %disp(bieq_p);
    
    for i = 1:n_seg
        if i ==5
            bieq_p_3(i,1) = -corridor_range(i,1);
        else
            bieq_p_3(i,1) = -corridor_range(i+1,1);
        end
    end
    bieq_p = [bieq_p;bieq_p_3];
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = [];
    bieq_v = [];

    k = 1;
    for l = 1:n_seg
        n = l-1;
        for i = 1:8
            m = i-1;
            if m>=k
                Aieq_v(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            else
                Aieq_v(l,i+8*n) = 0;
            end
        end
    end

    Aieq_v = Aieq_v*M;
    %disp(Aieq_v);  
    for i = 1:5
        n = i-1;      
        for j = 1:8
            bieq_v(j+n*8,1) =  v_max;
        end       
    end
    %disp(bieq_v);
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = [];
    bieq_a = [];
    
    k = 2;
    for l = 1:n_seg
        n = l-1;
        for i = 1:8
            m = i-1;
            if m>=k
                Aieq_a(l,i+8*n) = factorial(m)/factorial(m-k)*ts(1,1)^(m-k);
            else
                Aieq_a(l,i+8*n) = 0;
            end
        end
    end
    Aieq_a = Aieq_a*M;
    %disp(Aieq_a);
    for i = 1:5
        n = i-1;      
        for j = 1:8
            bieq_a(j+n*8,1) = a_max;
        end       
    end
    disp(bieq_a);
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    Aieq = Aieq_p;
    bieq = bieq_p;
end