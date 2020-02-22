clc
clear vars
close all
% Solving linear equations in order to obtain coefficiënts of a spline
% based path using the matlab backslash command.
% The backslash command automatically selects the correct solving method.
% In this case this will be a linear solver e.g. linsolve.

% k = i+1

syms a b c d e f ti tk pi vi ai pk vk ak

eqn = [-pi+a+b*ti+c*ti^2+d*ti^3+e*ti^4+f*ti^5 == 0
     -vi+b+2*c*ti+3*d*ti^2+4*e*ti^3+5*f*ti^4 == 0
     -ai+2*c+6*d*ti+12*e*ti^2+20*f*ti^3 == 0
     -pk+a+b*tk+c*tk^2+d*tk^3+e*tk^4+f*tk^5 == 0
     -vk+b+2*c*tk+3*d*tk^2+4*e*tk^3+5*f*tk^4 == 0
     -ak+2*c+6*d*tk+12*e*tk^2+20*f*tk^3 == 0];
 
 vars = [a b c d e f];
 
[A,b] = equationsToMatrix(eqn,vars);

disp('this is A')
A
disp('this is b')
b
disp('this is the solution')
sol = A\b

% Solution:

% a = -(2*pi*tk^5 - 2*pk*ti^5 + 10*pk*ti^4*tk - 2*ti*tk^5*vi + 2*ti^5*tk*vk + 20*pi*ti^2*tk^3 + ai*ti^2*tk^5 - 2*ai*ti^3*tk^4 + ai*ti^4*tk^3 - ak*ti^3*tk^4 + 2*ak*ti^4*tk^3 - ak*ti^5*tk^2 - 20*pk*ti^3*tk^2 + 10*ti^2*tk^4*vi - 8*ti^3*tk^3*vi + 8*ti^3*tk^3*vk - 10*ti^4*tk^2*vk - 10*pi*ti*tk^4)/(2*(ti - tk)^2*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));
%                             
% b = (2*ti^5*vk - 2*tk^5*vi + 2*ai*ti*tk^5 - 2*ak*ti^5*tk + 10*ti*tk^4*vi - 10*ti^4*tk*vk + 60*pi*ti^2*tk^2 - ai*ti^2*tk^4 - 4*ai*ti^3*tk^3 + 3*ai*ti^4*tk^2 - 3*ak*ti^2*tk^4 + 4*ak*ti^3*tk^3 + ak*ti^4*tk^2 - 60*pk*ti^2*tk^2 + 16*ti^2*tk^3*vi - 24*ti^3*tk^2*vi + 24*ti^2*tk^3*vk - 16*ti^3*tk^2*vk)/(2*(ti - tk)^2*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));
%                        
% c = -(ai*tk^5 - ak*ti^5 + 4*ai*ti*tk^4 + 3*ai*ti^4*tk - 3*ak*ti*tk^4 - 4*ak*ti^4*tk - 60*pk*ti*tk^2 - 60*pk*ti^2*tk + 36*ti*tk^3*vi - 24*ti^3*tk*vi + 24*ti*tk^3*vk - 36*ti^3*tk*vk - 8*ai*ti^2*tk^3 + 8*ak*ti^3*tk^2 - 12*ti^2*tk^2*vi + 12*ti^2*tk^2*vk + 60*pi*ti*tk^2 + 60*pi*ti^2*tk)/(2*(ti - tk)^2*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));
%                                     
% d = (20*pi*ti^2 + 20*pi*tk^2 + ai*ti^4 + 3*ai*tk^4 - 3*ak*ti^4 - ak*tk^4 - 20*pk*ti^2 - 20*pk*tk^2 - 8*ti^3*vi - 12*ti^3*vk + 12*tk^3*vi + 8*tk^3*vk + 4*ai*ti^3*tk - 4*ak*ti*tk^3 + 28*ti*tk^2*vi - 32*ti^2*tk*vi + 32*ti*tk^2*vk - 28*ti^2*tk*vk - 8*ai*ti^2*tk^2 + 8*ak*ti^2*tk^2 + 80*pi*ti*tk - 80*pk*ti*tk)/(2*(ti^2 - 2*ti*tk + tk^2)*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));
%  
% e = -(30*pi*ti + 30*pi*tk - 30*pk*ti - 30*pk*tk + 2*ai*ti^3 + 3*ai*tk^3 - 3*ak*ti^3 - 2*ak*tk^3 - 14*ti^2*vi - 16*ti^2*vk + 16*tk^2*vi + 14*tk^2*vk - 4*ai*ti*tk^2 - ai*ti^2*tk + ak*ti*tk^2 + 4*ak*ti^2*tk - 2*ti*tk*vi + 2*ti*tk*vk)/(2*(ti - tk)^2*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));
%                                                                                         
% f =  -(12*pk - 12*pi + 6*ti*vi + 6*ti*vk - 6*tk*vi - 6*tk*vk - ai*ti^2 - ai*tk^2 + ak*ti^2 + ak*tk^2 + 2*ai*ti*tk - 2*ak*ti*tk)/(2*(ti^2 - 2*ti*tk + tk^2)*(ti^3 - 3*ti^2*tk + 3*ti*tk^2 - tk^3));


     
  
