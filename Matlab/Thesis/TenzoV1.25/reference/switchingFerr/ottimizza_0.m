function J=ottimizza_0(ts)

global x0 C_0 nsample Tf y_rif dy_rif r g
dC=[0 0 1];

tspan_1 = [0:ts/nsample:ts];
tspan_2=[ts:(Tf-ts)/nsample:Tf];
t=[tspan_1,tspan_2];
if ts==0
    t=[0,tspan_2];
end
y_rif=1-exp(-g*t);
dy_rif=g*exp(-g*t);

if ts~=0
    [t,x_1]=ode45(@phase1_0,tspan_1,x0);
    x_ts=x_1(length(x_1),:);
    y_1=x_1*C_0';
else
    x_ts=x0;
    x_1=x0;
    y_1=x0*C_0';
end

if ts~=Tf
    [t,x_2]=ode45(@phase2_0,tspan_2,x_ts);
    y_2=x_2*C_0';
    
else
    x_2=x_ts;
    y_2=y_1;
end

err1=y_rif(1:length(y_1))-y_1';
err2=y_rif(length(y_1)+1:length(y_rif))-y_2';
L2e=(err1*err1')*ts/nsample+(err2*err2')*(Tf-ts)/nsample;

%dy1=(x_1*dC(:,2:3)')';
%dy2=(x_2*dC')';% dy vettore riga
%derr1=dy_rif(1:length(dy1))-dy1;
%derr2=dy_rif(length(dy1)+1:length(dy_rif))-dy2;


%L2de=(derr1*derr1')*ts/nsample+(derr2*derr2')*Tf/nsample;

J=L2e;%+L2de/abs(r-x0(1));

end