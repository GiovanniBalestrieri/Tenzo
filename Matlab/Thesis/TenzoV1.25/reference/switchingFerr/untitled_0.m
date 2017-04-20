clc
clear all
close all
%(0.15,3)
%[16*((2*pi/0.16)ˆ2)]
%[1 4*0.9*pi/0.16 (2*pi/0.16)ˆ2]
global A1 B1 C1 A2 B2 C2 r Tf nsample os_trsold x0 x0c w1num w1den n m x0t range resolution num den j_opt y_rif dy_rif g A1_0 B1_0 A2_0 B2_0 C_0 
%num=input('inserire coefficienti del numeratore del modello: ');
%den=input('inserire coefficienti del denominatore del modello: ');

g=8;
k=16*((2*pi/0.16)^2);
a=4*pi*0.9/0.16;
b=(2*pi/0.16)^2;
Kp=0.05;
Kp2=0.065;
Ki=0.5;
Ki2=2;



num=k;
den=[1 a b];
disp('Ricerca dei parametri indicativi Ki e Kp');
P=tf(num,den)
n=length(den)-1;
m=length(num)-1;
[w1num,w1den]=tfdata(P,'v');
w1num=w1num./w1den(1);
w1den=w1den./w1den(1);
%%PARAMETRI Kp e Ki indicativi
options = optimset('Algorithm','active-set','MaxIter',5e8,...
'MaxFunEvals',5e8,...
'TolX',1e-8,'TolFun',1e-8,'TolCon',1e-8);
%Tf=input('inserire tempo finale di simulazione: ');
Tf=1;
nsample=10000;
os_trsold=0;
Ao=-eye(2);
Bo=zeros(2,1);
Aeq = 0.*Ao;
beq = Bo.*0;
lb = [0 0];
ub = [100 100];
ci=[0.1 0.1];
x0=[0 0];
r=1;
y_rif=1;
dy_rif=0;
%  [X,Jval] = fmincon('parameters',ci,Ao,Bo,Aeq,beq,lb,ub,@nonlcon_2,options);
%  Kp=X(1)
%  Ki=X(2)
 

%%REALIZZAZIONE DEL SISTEMA A CICLO CHIUSO

disp('REALIZZAZIONE');
%Kp=input('inserire guadagno Kp: ');
%Ki=input('inserire guadagno Ki: ');
den=den./den(1);
num=num./den(1);
G1=Kp;
G2=tf([Kp Ki],[1 0]);
code=input('inserire 1 per realizzazione in forma di osservatore, 2 per la forma di controllore (0 di default): ');
if code==1
    A1=zeros(n,n);
    C1=zeros(1,n);
    A1(:,n)=-w1den(n+1:-1:2)'-Kp.*w1num(n+1:-1:2)';
    B1=Kp.*w1num(n+1:-1:2)';
    C1(1,n)=1;
    for i=2:n
        A1(i,i-1)=1;
    end
    A2=zeros(n+1,n+1);
    C2=zeros(1,n+1);
    A2(2:n+1,n+1)=-w1den(n+1:-1:2)'-Kp.*w1num(n+1:-1:2)';
    A2(2:n+1,1)=w1num(n+1:-1:2)';
    A2(1,n+1)=-Ki;
    B2=zeros(n+1,1);
    B2(2:n+1,1)=Kp.*w1num(n+1:-1:2)';
    B2(1,1)=Ki;
    C2(1,n+1)=1;
    for i=3:n+1
        A2(i,i-1)=1;
    end
end
if code==2
    A1=zeros(n,n);
    B1=zeros(n,1);
    A1(n,:)=-w1den(n+1:-1:2)'-Kp.*w1num(n+1:-1:2)';
    C1=w1num(n+1:-1:2);
    B1(n,1)=Kp;
    for i=1:n-1
        A1(i,i+1)=1;
    end
    A2=zeros(n+1,n+1);
    B2=zeros(n+1,1);
    A2(n+1,2:n+1)=-w1den(n+1:-1:2)'-Kp.*w1num(n+1:-1:2)';
    A2(n+1,1)=1;
    A2(1,2:n+1)=-Ki.*w1num(n+1:-1:2)';
    C2=zeros(1,n+1);
    C2(2:n+1)=C1;
    B2(1,1)=Ki;
    B2(n+1,1)=Kp;
    for i=2:n
        A2(i,i+1)=1;
    end
end
if code==0
    C1=zeros(1,n);
    C1(1,m+1)=1;
    A1=zeros(n);
    for i=1:n-1
        A1(i,i+1)=1;
    end
    A1(n,1:m)=-den(n+1:-1:n-m+2)-Kp*num(m+1:-1:2);
    A1(n,m+1:n)=-den(n-m+1:-1:2);
    A1(n,m+1)=A1(n,m+1)-Kp*num(1);
    B1=zeros(n,1);
    B1(n,1)=Kp*num(1);
    if m>0
        B1(m,1)=Kp*num(m+1)/(-den(n+1)-Kp*num(m+1));
    end
    for i=n-m+1:n-1
        for j=n-m:i-1
            B1(n-i,1)=B1(n-i,1)-B1(n-j,1)*(-den(2*n-m-j)-Kp*num(n-j));
        end
        B1(n-i,1)=(B1(n-i,1)+Kp*num(n-i+1))/(-den(n+1)-Kp*num(m+1));
    end

    C2=zeros(1,n+1);
    C2(1,m+2)=1;
    A2=zeros(n+1,n+1);
    A2(2:n+1,2:n+1)=A1;
    if m>0
        A2(m+1,1)=num(m+1)/(-den(n+1)-Kp*num(m+1));
    end
    for i=n-m+2:n
        for j=n-m+1:i-1
            A2(n-i,1)=A2(n-i,1)-A2(n-j,1)*(-den(2*n-m-j)-Kp*num(n-j));
        end
      A2(n-i,1)=(A2(n-i,1)+num(n-i+1))/(-den(n+1)-Kp*num(m+1));
    end
    A2(1,m+2)=-Ki;
    A2(n+1,1)=num(1);
    B2=zeros(n+1,1);
    B2(2:n+1,1)=B1;
    B2(1,1)=Ki;
end


disp('A1= ')
A1
disp('B1= ')
B1
disp('C1= ')
C1
disp('A2= ')
A2
disp('B2= ')
B2
disp('C2= ')
C2
f1=feedback(G1*P,1)
fs1=tf(ss(A1,B1,C1,0))
f2=feedback(G2*P,1)
fs2=tf(ss(A2,B2,C2,0))



A1_0=[0 -Ki 0; 0 0 1; k -b-k*Kp -a];
A2_0=[0 -Ki2 0; 0 0 1; k -b-k*Kp2 -a];
B1_0=[Ki; 0; k*Kp];
B2_0=[Ki2; 0; k*Kp2];
C_0=[0 1 0];


%%RICERCA DEL TEMPO OTTIMO DI SWITCHING

disp('TEMPO OTTIMO DI SWITCHING');
%r=input('inserire costante di riferimento da inseguire: ');
%x0=input('inserire condizioni iniziali: ');
%x0c=input('inserire valore iniziale dell integratore: ');
r=1;
x0=[0 0 0];
x0c=0;
[t1,Jval] = fmincon('ottimizza_0',0.1,-1,0,0,0,0,Tf,@nonlcon_2,options);

t1
Jval


%%GRAFICI DEI RISULTATI
disp('RISULTATI');
if(t1~=0)
    [tsim1,x1]=ode45(@phase1_0,[0 t1],x0);
    f1=x1*C_0';
    xt1=x1(length(f1),:);
else
    f1=0;
    xt1=x0;
    tsim1=0;
end
[tsim2,x2]=ode45(@phase2_0,[0 Tf],xt1);
f2=x2*C_0';
[tsimo,x2]=ode45(@phase2_0,[0 Tf],x0);
f2o=x2*C_0';
[tsimu,x1]=ode45(@phase1_0,[0 Tf],x0);
f1u=x1*C_0';



[tr1,rr1]=ode45(@ref,tsim1,1);
[tr2,rr2]=ode45(@ref,tsim2,rr1(length(rr1)));

[tro,rro]=ode45(@ref,tsimo,1);
[tru,rru]=ode45(@ref,tsimu,1);



figure(44);
plot([tr1',t1+tr2'],[1-rr1'-f1',1-rr2'-f2'],tro',1-rro'-f2o',tru',1-rru'-f1u');
legend('New Controller','Double Integrator','Old Integrator');


figure(1);
plot([tsim1',t1+tsim2'],[f1',f2'],tsimo,f2o',tsimu,f1u',[tr1',t1+tr2'],[1-rr1',1-rr2']);
legend('New Controller','Double Integrator','Old Controller','Reference');






%%RICERCA DELLE CURVE OTTIME SE n=1 O n=2


disp('CURVE OTTIME');
if(n==1||n==2)
    range=input('inserire range di variazione dello stato iniziale: (-range,+range): ');
    shift=input('inserire valori di shifting: (-range+shift,+range+shift): ');
    resolution=input('inserire risoluzione dello stato iniziale: ');
    shiftc=input('inserire valore centrale di x0c: ');
    rangec=input('inserire range di x0c: ');
if rangec~=0
    resolutionc=input('inserire risoluzione dell integratore: ');
else
    resolutionc=1;
end
x0t=zeros(n,((2*max(range))/min(resolution)+1));
len=1;
for i=1:n
    x0t(i,1:2*range(i)/resolution(i)+1)=[-range(i)+shift(i):resolution(i):range(i)+shift(i)];
    len=len*(range(i)/resolution(i)+1);
end
x1t=zeros(1,len);
x2t=zeros(1,len);
t1t=zeros(1,len);
y0t=zeros(1,len);
yt1=zeros(1,len);
x0ct=[-rangec+shiftc:resolutionc:rangec+shiftc];
j_t=zeros((2*range(1))/resolution(1)+1,(2*range(2))/resolution(2)+1);
j_pi=zeros((2*range(1))/resolution(1)+1,(2*range(2))/resolution(2)+1);


if(n==2)
cnt=1;
for i=1:(2*range(1))/resolution(1)+1
    for j=1:(2*range(2))/resolution(2)+1
        for k=1:(2*rangec)/resolutionc+1
            x01=x0t(1,i);
            x02=x0t(2,j);
            x0=[x01 x02];
            y0t(cnt)=x0*C1';
            x0c=x0ct(k);
            [t1,Jval] = fmincon('ottimizza',0.1,-1,0,0,0,0,Tf,@nonlcon_2,options);
            if t1~=0
                [tsim1,x1]=ode45(@phase1,[0:t1/nsample:t1],x0);
                f1=x1*C1';
                yt1(cnt)=f1(length(f1));
                x1t(cnt)=x1(length(f1),1);
                x2t(cnt)=x1(length(f1),2);
            else
                yt1(cnt)=x0*C1';
                x1t(cnt)=x01;
                x2t(cnt)=x02;
            end
        t1t(cnt)=t1;
        cnt=cnt+1;
        j_t(j,i)=Jval;
        end
    end
end
j_t_1=j_t;
j_opt=j_t;

for i=1:(2*range(1))/resolution(1)+1
    for j=1:(2*range(2))/resolution(2)+1
        x01=x0t(1,i);
        x02=x0t(2,j);
        x0=[x01 x02];
        x0c=0;
       % j_t(j,i)=Switch_Area([70 0.6]);
       % j_pi(j,i)=PI();
    end
end
figure(8);
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_t,j_t);
hold on;
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_pi,100-j_pi);
hold on;
figure(9)
hold on;
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_opt,j_opt);
hold on;
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_t,100-j_t);
cmap = colormap;
nn=1;
k=0;
dec=0.04*(21/(max(2*range(1)/resolution(1)+1,2*range(2)/resolution(2)+1)));
for i=0:2*dec:100
    for j=1:(2*range(2))/resolution(2)+1
        if mod(i,((2*range(2))/resolution(2)+1)*2*dec)==0
            k=k+dec;
        end
        if mod(i,((2*range(2))/resolution(2)+1)*2*dec)<((2*range(2))/resolution(2)+1)*dec
            cmap(nn,1:3)=[mod(i,((2*range(2))/resolution(2)+1)*dec) 1-mod(i,((2*range(2))/resolution(2)+1)*dec) 1-k];
        else
            cmap(nn,1:3)=[1-k mod(i,((2*range(2))/resolution(2)+1)*dec) 1-mod(i,((2*range(2))/resolution(2)+1)*dec)];
        end
    nn=nn+1;
    end
end

figure(3)
hold on;
for i=1:(2*range(1))/resolution(1)+1
    for j=1:(2*range(2))/resolution(2)+1
        plot(x0t(1,i),x0t(2,j),'Marker','+','LineStyle','none','Color',cmap(mod((i-1)*((2*range(1))/resolution(1)+1)+j,length(cmap))+1,:));
    end
end
figure(4)
hold on;
for i=1:cnt-1
    plot(x1t(i),x2t(i),'Marker','+','LineStyle','none','Color',cmap(mod(i,length(cmap))+1,:));
end
figure(5)
hold on
for i=1:cnt-1
    plot(y0t(i),t1t(i),'Marker','+','LineStyle','none','Color',cmap(mod(i,length(cmap))+1,:));
end
figure(6)
hold on
for i=1:cnt-1
    plot(y0t(i),yt1(i),'Marker','+','LineStyle','none','Color',cmap(mod(i,length(cmap))+1,:));
end

%%CALCOLO DELLE SOGLIE OTTIME DI SWITCHING PER SWITCH-AREA CONTROLLER

cc=input('Inserire 1 per calcolare le soglie ottime di switching: ');
options = optimset('Algorithm','active-set','MaxIter',5e8,...
'MaxFunEvals',5e8,...
'TolX',1e-8,'TolFun',1e-8,'TolCon',1e-8);
if(cc==1)
    At=-eye(2);
    Bt=zeros(2,1);
    Aeqt = 0.*Ao;
    beqt = Bo.*0;

    lbt = [0 0];
    ubt = [1 100];
    cit=[1 100];
    [X,Jval] = fminsearch('best_threshold',cit);
    sy=X(1)
    sdy=X(2)
    Jval
for i=1:(2*range(1))/resolution(1)+1
    for j=1:(2*range(2))/resolution(2)+1
        x01=x0t(1,i);
        x02=x0t(2,j);
        x0=[x01 x02];
        x0c=0;
        j_t_1(j,i)=Switch_Area([0.6 70]);
        j_t(j,i)=Switch_Area([sy sdy]);
    end
end
figure(26)
hold on;
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_t_1 ,j_t_1);
hold on;
surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_t,100-j_t);
end
%%CALCOLO DEI PARAMETRI MIGLIORI Ki e Kp ADATTI ALLO SWITCHING
cc=input('inserire 1 se si vuole cercare i guadagni ottimi: ');
if cc==1
    Ao=-eye(2);
    Bo=zeros(2,1);
    Aeq = 0.*Ao;
    beq = Bo.*0;
    lb = [0 0];
    ub = [1 10];

    ci=[0.05 1.8];
    [X,Jval] = fmincon('full_optim',ci,Ao,Bo,Aeq,beq,lb,ub,@nonlcon_2,options);
    Kp=X(1);
    Ki=X(2);
    C1=zeros(1,n);
    C1(1,m+1)=1;
    A1=zeros(n);
    for i=1:n-1
        A1(i,i+1)=1;
    end
    A1(n,1:m)=-den(n+1:-1:n-m+2)-Kp*num(m+1:-1:2);
    A1(n,m+1:n)=-den(n-m+1:-1:2);
    A1(n,m+1)=A1(n,m+1)-Kp*num(1);
    B1=zeros(n,1);
    B1(n,1)=Kp*num(1);
    if m>0
        B1(m,1)=Kp*num(m+1)/(-den(n+1)-Kp*num(m+1));
    end
    for i=n-m+1:n-1
        for j=n-m:i-1
            B1(n-i,1)=B1(n-i,1)-B1(n-j,1)*(-den(2*n-m-j)-Kp*num(n-j));
        end
        B1(n-i,1)=(B1(n-i,1)+Kp*num(n-i+1))/(-den(n+1)-Kp*num(m+1));
    end
    C2=zeros(1,n+1);
    C2(1,m+2)=1;
    A2=zeros(n+1,n+1);
    A2(2:n+1,2:n+1)=A1;
    if m>0
        A2(m+1,1)=num(m+1)/(-den(n+1)-Kp*num(m+1));
    end
    for i=n-m+2:n
        for j=n-m+1:i-1
            A2(n-i,1)=A2(n-i,1)-A2(n-j,1)*(-den(2*n-m-j)-Kp*num(n-j));
        end
        A2(n-i,1)=(A2(n-i,1)+num(n-i+1))/(-den(n+1)-Kp*num(m+1));
    end
    A2(1,m+2)=-Ki;
    A2(n+1,1)=num(1);
    B2=zeros(n+1,1);
    B2(2:n+1,1)=B1;
    B2(1,1)=Ki;
    for i=1:(2*range(1))/resolution(1)+1
        for j=1:(2*range(2))/resolution(2)+1
            for k=1:(2*rangec)/resolutionc+1
                x01=x0t(1,i);
                x02=x0t(2,j);
                x0=[x01 x02];
                y0t(cnt)=x0*C1';
                x0c=x0ct(k);
                [t1,Jval] = fmincon('ottimizza',0.1,-1,0,0,0,0,Tf,@nonlcon_2,options);
                if t1~=0
                    [tsim1,x1]=ode45(@phase1,[0:t1/nsample:t1],x0);
                    f1=x1*C1';
                    yt1(cnt)=f1(length(f1));
                    x1t(cnt)=x1(length(f1),1);
                    x2t(cnt)=x1(length(f1),2);
                else
                    yt1(cnt)=x0*C1';
                    x1t(cnt)=x01;
                    x2t(cnt)=x02;
                end
                t1t(cnt)=t1;
                cnt=cnt+1;
                j_t(j,i)=Jval;
            end
        end
    end
    figure(19)
    hold on;
    surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_opt,j_opt);
    hold on;
    surf(x0t(1,1:(2*range(1))/resolution(1)+1),x0t(2,1:(2*range(2))/resolution(2)+1),j_t,100-j_t);
    end
end
if(n==1)
    for i=1:(2*range)/resolution+1
        x0=x0t(1,i);
        [t1,Jval] = fmincon('ottimizza',0.1,-1,0,0,0,0,Tf,@nonlcon_2,options);
        [tsim1,x1]=ode45(@phase1,[0:t1/nsample:t1],x0);
        f1=x1*C1';
        x1t(i)=x1(length(f1),1);
        t1t(i)=t1;
    end

    figure(4)
    plot(x0t,x1t);
    figure(5)
    plot(x0t,t1t);
end
end