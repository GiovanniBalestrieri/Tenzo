A = [1.1269   -0.4940    0.1129;
     1.0000         0         0;
          0    1.0000         0];

B = [-0.3832;
      0.5919;
      0.5191];

C = [1 0 0];

n = size(A,2)

disp('Raggiungibilità');
disp(rank(ctrb(A,B)))

disp('Controllabilità');
disp(rank(ctrb(A,C')))

Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
step(Plant)

Q = 1;
R = 1;
[kalmf,L,P,M] = kalman(Plant,Q,R);
