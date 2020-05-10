function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = zeros(n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %
        %
        %
        %
        for i = 0:3
            M_k(i+1,i+1) = factorial(i);
        end
        
        for i= 4:7
            order=i-4;
            for j=1:n_order+1
                if j-1-order<0
                         M_k(i+1, j)=0;
                    else
                        M_k(i+1, j) = factorial(j-1)/factorial(j-order-1)*ts(k)^(j-1-order);
                    end                
            end
        end
%         for i = 0:int32((n_order+1)/2)-1
%             M_k(i+1,i+1) = factorial(i);
%         end
%         
%         for i = int32((n_order+1)/2):n_order
%             order = i-int32((n_order+1)/2);
%             for j=1:n_order+1
%                    if j-1-order<0
%                         M_k(i+1, j)=0;
%                    else
%                        M_k(i+1, j) = factorial(j-1)/factorial(j-order-1)*ts(k)^(j-1-order);
%                    end
%             end        
%         end
        M = blkdiag(M, M_k);
    end
