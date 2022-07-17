% without SAFE CORRIDOR
function [Aieq, bieq] = getAbieq2(n_seg, n_order, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
% % %     Aieq_p = zeros(n_seg*(n_order+1)*2, n_all_poly);
% % %     bieq_p = zeros(n_seg*(n_order+1)*2, 1);
% % %     for k=1:n_seg
% % %        for i=1:(n_order+1)
% % %           Aieq_p(i+16*(k-1), i+8*(k-1)) = 1*ts(k);
% % %           Aieq_p(8+i+16*(k-1), i+8*(k-1)) = -1*ts(k);
% % %           bieq_p(i+16*(k-1)) = corridor_range(k,2);
% % %           bieq_p(8+i+16*(k-1)) = -corridor_range(k,1);
% % %        end
% % %     end
    Aieq_p = [];
    bieq_p = [];
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = zeros(n_seg*(n_order)*2, n_all_poly);
    bieq_v = zeros(n_seg*(n_order)*2, 1);
    for k=1:n_seg
        for i=1:(n_order)
            Aieq_v(i+14*(k-1), i+8*(k-1):i+8*(k-1)+1) = n_order*[-1, 1];
            Aieq_v(7+i+14*(k-1), i+8*(k-1):i+8*(k-1)+1) = n_order*[-1, 1];
            bieq_v(i+14*(k-1)) = v_max;
            bieq_v(7+i+14*(k-1)) = v_max;
        end
    end
    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros(n_seg*(n_order-1)*2, n_all_poly);
    bieq_a = zeros(n_seg*(n_order-1)*2, 1);
    for k=1:n_seg
        s = ts(k);
        for i=1:(n_order-1)
            Aieq_a(i+12*(k-1), i+8*(k-1):i+8*(k-1)+2)=n_order*(n_order-1)/s*[1, -2, 1];
            Aieq_a(6+i+12*(k-1), i+8*(k-1):i+8*(k-1)+2)=n_order*(n_order-1)/s*[1, -2, 1];
            bieq_a(i+12*(k-1)) = a_max;
            bieq_a(6+i+12*(k-1)) = a_max;
        end
    end
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
% %     Aieq = Aieq_p;
% %     bieq = bieq_p;
end