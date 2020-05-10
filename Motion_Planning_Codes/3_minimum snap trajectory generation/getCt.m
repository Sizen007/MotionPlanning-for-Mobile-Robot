function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %
    Ct = zeros((n_order+1)*n_seg, (n_order+1)/2*(n_seg-1));
    
    for i=1:4
        Ct(i,i) = 1;
    end
    
    for n=1:n_seg-1
        Ct(4+8*(n-1)+1, 4+n) = 1;
        Ct(4+8*(n-1)+2, 8+n_seg-1+3*(n-1)+1)=1;
        Ct(4+8*(n-1)+3, 8+n_seg-1+3*(n-1)+2)=1;
        Ct(4+8*(n-1)+4, 8+n_seg-1+3*(n-1)+3)=1;
        
        Ct(4+8*(n-1)+5, 4+n) = 1;
        Ct(4+8*(n-1)+6, 8+n_seg-1+3*(n-1)+1)=1;
        Ct(4+8*(n-1)+7, 8+n_seg-1+3*(n-1)+2)=1;
        Ct(4+8*(n-1)+8, 8+n_seg-1+3*(n-1)+3)=1;
    end
    
    for i=1:4
        Ct(end-3, 4+n_seg-1+1) = 1;
        Ct(end-2, 4+n_seg-1+2) = 1;
        Ct(end-1, 4+n_seg-1+3) = 1;
        Ct(end, 4+n_seg-1+4) = 1;
    end
    
end