
function [decodedParameters] = positionEstimatorTraining(training_data)

NumTrain=size(training_data,1);
NumDir=8;
NeurUnits=98;
DimStateVec=6;
MaxTime=zeros(NumTrain,NumDir);
StartingTime=300;
vectg=[18 27 31 44 69 81 84 90 93 97];
trial=training_data;

%% Training of the estimator

%The following structure "Dir" will cointain a realization of the system
%x(k+1)=Ax(k)+w(t)    w(t)~N(0,W)
%z(k)=H*x(k)+q(t)     q(t)~N(0,Q)
%for each trials each direction. x(k) is the state
%[x,y,velx,vely,accx,accy]' and z(k) is the neural activity (instant k)


%Create a structure that contains all the A,W,H,Q for all the trials and
%angles

Dir = struct('A',zeros(DimStateVec,DimStateVec),'H',zeros(NeurUnits,DimStateVec),'W',zeros(DimStateVec,DimStateVec),'Q',zeros(NeurUnits,NeurUnits),'P',zeros(DimStateVec,DimStateVec),'dir',zeros(1,1),'maxtime',zeros(1,1));%,'spikes',zeros(10,600));

%For each trials/angles save its effective lenght in Dir(n,k).maxtime
for n=1:NumTrain,
    for k=1:NumDir,
        MaxTime(n,k)=length(trial(n,k).spikes(1,:))-100;
        Dir(n,k).maxtime=MaxTime(n,k);
    end
end

% Compute xk, xkp1, zk in order to obtain the means of the A,W,H,Q matrices and
% store them in Dir(n,k)
for n=1:NumTrain,
    for k=1:NumDir,
        
        %Initialization of x(k) and x(k+1)
        
        xk=zeros(DimStateVec,floor((MaxTime(n,k)-StartingTime)/20)+1);
        xkp1=zeros(DimStateVec,floor((MaxTime(n,k)-StartingTime)/20)+1);

        %Building of x(k)=[x,y,0,0,0,0]
        c=0;
        for t=0:20:MaxTime(n,k)-StartingTime,
            c=c+1; %instant T
            xk(1,c)=trial(n,k).handPos(1,t+StartingTime);
            xk(2,c)=trial(n,k).handPos(2,t+StartingTime);
        end
        
        %Computation of velx vely accx accy
        xk(3,:)=[xk(1,2:end) xk(1,end)]-xk(1,1:end);
        xk(4,:)=[xk(2,2:end) xk(2,end)]-xk(2,1:end);
        xk(5,:)=[xk(3,2:end) xk(3,end)]-xk(3,1:end);
        xk(6,:)=[xk(4,2:end) xk(4,end)]-xk(4,1:end);
        
        %Building of x(k+1)
        xkp1(1,1:end)=[xk(1,2:end) xk(1,end)];
        xkp1(2,1:end)=[xk(2,2:end) xk(2,end)];
        xkp1(3,:)=[xkp1(1,2:end) xkp1(1,end)]-xkp1(1,1:end);
        xkp1(4,:)=[xkp1(2,2:end) xkp1(2,end)]-xkp1(2,1:end);
        xkp1(5,:)=[xkp1(3,2:end) xkp1(3,end)]-xkp1(3,1:end);
        xkp1(6,:)=[xkp1(4,2:end) xkp1(4,end)]-xkp1(4,1:end);
        
        %Initialization of z(k)
        zk=zeros(NeurUnits,floor((MaxTime(n,k)-StartingTime)/20)+1);
        
        %Computation of z(k)
        for i=1:NeurUnits,
            c=0;
            for t=0:20:MaxTime(n,k)-StartingTime,
                c=c+1;
                if (MaxTime(n,k)-StartingTime)-t>=20, 
                    zk(i,c)=sum(trial(n,k).spikes(i,1+t+StartingTime:t+20+StartingTime));
                else
                    zk(i,c)=sum(trial(n,k).spikes(i,t+1+StartingTime:end));
                end
            end
        end 

        %Least Square Estimation of A and W. A relates the state at the
        %previous time step to the state at the current time step, W is the
        % process noise covariance matrix
        
        A=(xkp1*xk')/(xk*xk'); %2x2 ok
        
        %W=E[(x(k+1)-A*x(k))*(x(k+1)-A*x(k))']
        
        W=((xkp1-A*xk)*((xkp1-A*xk)'))/length(xkp1); 

        %Least Square Estimation of H and Q. The encoding matrix
        % H can be expressed as (zk*xk')/(xk*xk'). The residuals (zk-H*xk) are 
        %defined as the difference between the actual values and the
        %fitted values: (zk-H*xk). The measurement noise covariance matrix
        %Q is the expected value(mean) of the squared residuals
        
        H=(zk*xk')/(xk*xk');
        
        %Q=E[(z(k)-H*x(k))*(z(k)-H*x(k))']
        
        Q=((zk-H*xk)*((zk-H*xk)'))/length(zk); 
        
        %Store the matrices in the structure Dir 
        Dir(n,k).A=A;
        Dir(n,k).H=H;
        Dir(n,k).W=W;
        Dir(n,k).Q=Q;

    end
end

for k=1:NumDir,
    for n=1:NumTrain,
      Dir(n,k).spikes=trial(n,k).spikes(vectg,201:320); %10x
    end
end

% Mean of the matrix with the same direction. 
% The 8 (k) final matrices are stored in Dir(1,k)
for k=1:NumDir,
    for d=2:NumTrain,
        Dir(1,k).A=Dir(1,k).A+Dir(d,k).A;
        Dir(1,k).H=Dir(1,k).H+Dir(d,k).H; 
        Dir(1,k).W=Dir(1,k).W+Dir(d,k).W;   
        Dir(1,k).Q=Dir(1,k).Q+Dir(d,k).Q;
    end
    Dir(1,k).A=Dir(1,k).A/NumTrain;
    Dir(1,k).H=Dir(1,k).H/NumTrain; 
    Dir(1,k).W=Dir(1,k).W/NumTrain;  
    Dir(1,k).Q=Dir(1,k).Q/NumTrain;
end

decodedParameters=Dir;

end