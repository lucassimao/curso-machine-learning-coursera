function [J, grad] = costFunctionReg(theta, X, y, lambda)
%COSTFUNCTIONREG Compute cost and gradient for logistic regression with regularization
%   J = COSTFUNCTIONREG(theta, X, y, lambda) computes the cost of using
%   theta as the parameter for regularized logistic regression and the
%   gradient of the cost w.r.t. to the parameters. 

% Initialize some useful values
m = length(y); % number of training examples

% You need to return the following variables correctly 
J = 0;
grad = zeros(size(theta));

% ====================== YOUR CODE HERE ======================
% Instructions: Compute the cost of a particular choice of theta.
%               You should set J to the cost.
%               Compute the partial derivatives and set grad to the partial
%               derivatives of the cost w.r.t. each parameter in theta

%26x1 100x26
%1 x 26 . 26.100

for i=1:m
  h0 = sigmoid(theta' * X(i,:)' );
  J += -y(i) * log(h0) - (1 - y(i))*log(1 - h0);
endfor
J /= m;
J += (lambda/(2*m)) * sum(theta(2:end,1) .^ 2);


for j=1:size(grad,1)
  for i=1:m
    h0 = sigmoid(theta' * X(i,:)' );
    grad(j) += (h0 - y(i)) * X(i,j);
  endfor
    grad(j) /= m;
    if j > 1
      grad(j) += (lambda/m) * theta(j);
     endif;
endfor



% =============================================================

end
