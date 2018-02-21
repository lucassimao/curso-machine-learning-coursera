% X     Conjunto de amostrados do modelo
% N     nº de variavaies de cada entrada do modelo
% M     nº de amostradas do modelo ( length(X) == M )
% Theta vetor com os coeficientes da funçao do modelo. Tem sempre N+1 elementos
1;

% x eh Nx1
% theta (N+1) x 1
function h = hipotese(theta,x) 
  x = [ 1 ; x]; % Adicionando 1 como 1º elemento
  h = theta' * x;
endfunction
  
% theta eh um vetor (N+1)x1
function J = costFunction(theta,X,y)
    J = 0;
    m = length(X);
    for i=1:m
      J += (hipotese(theta,X(i)) - y(i)) ^ 2;
    endfor
    J /= (2*m);    
endfunction

function drawPointsWithLines(X,y)
  figure; hold on;
    plot(X, y, 'ko', 'MarkerFaceColor', 'y','MarkerSize', 7);
    xlabel ("X");
    ylabel ("y");
        
    [minX,idxMinX] = min(X);
    [maxX,idxMaxX] = max(X);
    
    interval = .95:0.1:1.5;
    h = zeros(length(interval),1);
    legends =  cellstr(interval);
    for i=1:length(interval)
        theta = [ -3.8958; interval(i)]; %1.1930
        yMinPredict = theta' * [1 ; minX]; % y(idxMinX)
        yMaxPrediction = theta' * [1 ; maxX]; % y(idxMaxX)

        h(i) = plot([minX; maxX], [ yMinPredict ; yMaxPrediction ], 'LineWidth', 2);
        legends{i} =  sprintf("Theta(%d,%d) J = ",theta(1),theta(2),costFunction(theta,X,y));
        legend(h, legends);
    endfor
  hold off;
endfunction

function drawCostFunction(X,y)
  figure; hold on;
    title('Grafico do custo da hipotese em funcao do theta1 com theta0=-.9');
    xlabel('Theta1');
    ylabel('Cost');
    
    interval = .5:0.05:1.2;
    theta1s = [];
    costs = [];
    for i=1:length(interval)
      theta = [-0.9; interval(i)];
      theta1s = [ interval(i), theta1s];
      costs =  [ costFunction(theta,X,y), costs];
    endfor
    plot(theta1s,costs, 'LineWidth', 2);
    legend('Cost of hypothesis by theta(0, theta1)')
  hold off;
endfunction

function [theta,costHistory] = gradientDescent(initital_theta,X,y,alpha,MaxInterations)
  theta = initital_theta;
  m = length(X);
  costHistory = zeros(MaxInterations,1);
  aux = 1;
  
  for aux=1:MaxInterations
    temp = zeros(size(theta));
    %display(sprintf('Iteracao %d',aux));
    
    for j=1:length(theta)    
        % somatorio
        %display(sprintf('j %d',j));
        for i=1:m
          %display(sprintf('m %d',i));
          if j > 1 
            temp(j) += (hipotese(theta,X(i)) - y(i))*X(i,j-1);
          else
            temp(j) += hipotese(theta,X(i)) - y(i);
          endif
        endfor

        temp(j) = theta(j)  -alpha * (1/m) * temp(j);
    endfor
    theta = temp;
    display(sprintf('Iteracao %d %d %d %d',aux,theta(1),theta(2),costFunction(theta,X,y)));
    costHistory(aux) = costFunction(theta,X,y);
    aux += 1;
  endfor
endfunction


function [theta,costHistory] = gradientDescentThresould(initital_theta,X,y,alpha,threshouldCost)
  theta = initital_theta;
  m = length(X);
  costHistory = [];
  
  while ( length(costHistory) < 2 || (costHistory(end-1) - costHistory(end)) > threshouldCost  )
    temp = zeros(size(theta));
    %display(sprintf('Iteracao %d',aux));
    
    for j=1:length(theta)    
        % somatorio
        %display(sprintf('j %d',j));
        for i=1:m
          %display(sprintf('m %d',i));
          if j > 1 
            temp(j) += (hipotese(theta,X(i)) - y(i))*X(i,j-1);
          else
            temp(j) += hipotese(theta,X(i)) - y(i);
          endif
        endfor

        temp(j) = theta(j)  -alpha * (1/m) * temp(j);
    endfor
    theta = temp;
    cost = costFunction(theta,X,y);
    costHistory = [costHistory ; cost]; 
    display(sprintf('Iteracao %d %d %d %d',length(costHistory),theta(1),theta(2),costFunction(theta,X,y)));
  endwhile
endfunction


function drawCostHistoryThroughGradient(history,MaxInterations)
  figure; hold on;
    title('Grafico de historia do custo em funcao do numero de iteracoes do gradient descent');
    xlabel('Iteracoes');
    ylabel('Cost');
    
    X = 1:MaxInterations;
    plot(X',history, 'LineWidth', 2);
    legend('Cost of hypothesis by theta(0, theta1)')
  hold off;
endfunction

function theta = computeThetaWithNormalForm(X,y)
    _1stcolumn = ones(length(X),1);
  X= [ _1stcolumn X];

  theta = pinv(X'*X)*(X'*y);
endfunction

data = load('../ex1/ex1data1.txt');
X = data(:,1);
y = data(:,2);
N = size(X,2);
initital_theta = [0 ;1.3];

display(computeThetaWithNormalForm(X,y));


%h = hipotese(initital_theta,X(1));
%J = costFunction(initital_theta,X,y);
%drawPointsWithLines(X,y);
%drawCostFunction(X,y);
%[theta,costHistory] = gradientDescentThresould([10;100],X,y,0.01,1e-5);
%disp(theta);
%drawCostHistoryThroughGradient(costHistory,length(costHistory));


