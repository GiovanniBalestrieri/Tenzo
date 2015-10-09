% Nyquist test

H = zpk([2 2 -3],[-2 -2 -2 -1],1)
IPGH = 1 + H;
nyquist(IPGH);


cLoop = feedback(IPGH,1)
step(cLoop)

%% Decomposizione valori singolari
A = diag([1 2 3 4])
%A'*A
s = svd(A)

% (x-1)(x-2)(x+3)
u = [1  0  -7  6]
A = compan(u)
eig = eig(A)

Sing = svd(A)
max(Sing)
% prova formula maxSingVal(A) = max_i sqrt(lambda-i*(A'*A)
sqrt(eig(1)*(A'*A))
sqrt(eig(2)*(A'*A))
sqrt(eig(3)*(A'*A))


% should be the same as Sing
sqrt(eig(A'*A))

Sing = svd(A)
% massimo valore singolare
max(Sing)