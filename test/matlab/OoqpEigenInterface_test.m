% OOQP Notation: min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f, and l <= x <= u.

clear all
clc

%% Quadratic Programming

Q = [1 -1; -1 2]; 
c = [-2; -6];
C = [1 1; -1 2; 2 1];
f = [2; 2; 3];
A = zeros(2,2);
b = zeros(2,1);
l = zeros(2,1);
u = [1000; 100];

opts = optimoptions('quadprog','Algorithm','interior-point-convex','Display','testing');

% No constraints
[x,fval,exitflag,output,lambda] = quadprog(Q,c,[],[],[],[],[],[],[],opts);
x,fval,exitflag

% With constraints
[x,fval,exitflag,output,lambda] = quadprog(Q,c,C,f,[],[],l,u,[],opts);
x,fval,exitflag

%% Linear Programming

Q = zeros(3, 3);
c = [-5; -4; -6];
C =  [1 -1  1
      3  2  4
      3  2  0];
f = [20; 42; 30];
l = zeros(3,1);

% No constraints (negative)
[x,fval,exitflag,output,lambda] = linprog(-c,[],[],[],[],l);
x,fval,lambda.ineqlin,lambda.lower

% With constraints
[x,fval,exitflag,output,lambda] = linprog(c,C,f,[],[],l);
x,fval,lambda.ineqlin,lambda.lower