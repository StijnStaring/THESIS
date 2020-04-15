function [deriv] = derivative(array,dt)
    if size(array,1) ~= 1 && size(array,2) ~=1
        error('Input should be an array with dimension 1!')
    end
    deriv = zeros(1,length(array));
    for i = 1:1:length(array)
        if i == 1
            deriv(i) = (array(2)-array(1))/dt;
        elseif i == length(array)
            deriv(i) = (array(end)-array(end-1))/dt;
            
        else
            deriv(i) = (array(i+1) - array(i-1))/(2*dt);
        end
    end
end 
