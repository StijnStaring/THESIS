function [del_w, exception, w_new, update_out] = RPROP(grad_curr,cas,length,update,w_curr,del_w_prev)
    
    % definition of parameters:
    n_pos = 1.2;
    n_neg = 0.5;
    del_max = 1.0;
    del_min = 1e-7;
    update_out = zeros(1,length);
    w_new = zeros(1,length);
    del_w = zeros(1,length);
    exception = zeros(1,length);

    % algorithm
    for i = 1:1:length
        
        % A fixed value is tried in order to remove the scaling factor when
        % learning the weights
        
        % if i == index_fixed_value:
        %     w_new(i) = w_curr(i)
        % else:

        if cas(i) == 1
            update(i) = min(update(i)*n_pos,del_max);
            del_w(i) = -sign(grad_curr(i))*update(i);
            w_new(i) = w_curr(i) + del_w(i);
            exception(i) = 0;


        elseif cas(i) == 2
            update(i) = max(update(i) * n_neg, del_min);
            del_w(i) = - del_w_prev(i);
            w_new(i) = w_curr(i) +del_w(i);
            exception(i) = 1;


        elseif cas(i) == 3
            del_w(i) = -sign(grad_curr(i)) * update(i);
            w_new(i) = w_curr(i) + del_w(i);
            exception(i) = 0;
            
        end
    end

    % do check if weight becomes negative
    for i = 1:1:length
        if w_new(i) < 0
            w_new(i) = 1e-8;
            update(i) = abs(1e-8 - w_curr(i));
            del_w(i) = -sign(grad_curr(i)) * update(i); 
            
            fprintf('\n')
            fprintf('A negative weight spotted - weight %i!',i)
            fprintf('\n')
        end      
    end
    
    for i = 1:1:length
        update_out(i) = update(i); % now is pointing to local defined update
    end
    

    
end