clear all
clc

A = [1 -1; -4 -3; 2 0]; 
S = diag([3, 2, 0.69]);
W = diag([0.5, 0.2]);
b = [3; 6; 9];

Q = A'*S*A + W;
c = -A'*S*b;
%%
opts = optimoptions('quadprog','Algorithm','interior-point-convex','Display','testing');

[x,fval,exitflag,output,lambda] = quadprog(Q,c,[],[],[],[],[],u,[],opts);
x,fval,exitflag