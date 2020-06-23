function [] = opti_weight_update(n,curr_data,file)

% n = the amount of features and equal to six 
dth = 1e-4;
jacob1= zeros(1,n);
hess1 = zeros(n,n);
jacob2= zeros(1,n);
hess2 = zeros(n,n);
jacob3= zeros(1,n);
hess3 = zeros(n,n);
jacob4= zeros(1,n);
hess4 = zeros(n,n);
jacob5= zeros(1,n);
hess5 = zeros(n,n);
jacob6= zeros(1,n);
hess6 = zeros(n,n);

if n ~= 6
    error("Error: the  amount of features doesn't equal six. Add extra intitial gradient and hessian matrices in opti_weight_update.")
end

% collect numerical points
for i = 1:1:n
    num_stock1 = zeros(1,2); % first value k+1 second value k-1
    num_stock2 = zeros(1,2);
    num_stock3 = zeros(1,2);
    num_stock4 = zeros(1,2);
    num_stock5 = zeros(1,2);
    num_stock6 = zeros(1,2);
    
    for j = 1:1:2
        [features_out] = varied_weights(theta,curr_data,rec,N,file); % features out should be 1xn and vary the same weight
        % and first vary k+1  and then k-1. Make use of the SQP method to
        % exploit the good intitial guess.
        num_stock1(1,j) =  features_out(1,1);
        num_stock2(1,j) =  features_out(1,2);
        num_stock3(1,j) =  features_out(1,3);
        num_stock4(1,j) =  features_out(1,4);
        num_stock5(1,j) =  features_out(1,5);
        num_stock6(1,j) =  features_out(1,6);
    end
    % approximate the gradients
    jacob1(1,i) = (num_stock1(1,1)-num_stock1(1,2))/(2*dth);
    jacob2(1,i) = (num_stock2(1,1)-num_stock2(1,2))/(2*dth);
    jacob3(1,i) = (num_stock3(1,1)-num_stock3(1,2))/(2*dth);
    jacob4(1,i) = (num_stock4(1,1)-num_stock4(1,2))/(2*dth);
    jacob5(1,i) = (num_stock5(1,1)-num_stock5(1,2))/(2*dth);
    jacob6(1,i) = (num_stock6(1,1)-num_stock6(1,2))/(2*dth);
end

% approximate the hessians


    
    
    
    
    
    
    
end
