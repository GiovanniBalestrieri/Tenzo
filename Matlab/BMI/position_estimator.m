function [decodedPos, newpar] = position_estimator(Dir, past_trials)

    DimStateVec=6;
    StartingTime=300;
    ntest=1;
    MaxTimeTest=length(past_trials(ntest).spikes(1,:));
    t=MaxTimeTest-StartingTime;
    
    
    
    if t<=20,
        [reaching_angles] = classifier_k(Dir, past_trials);
        Dir(1,1).dir=reaching_angles;
    end
    
    ktest=Dir(1,1).dir; % ktest=direction of mouvement

    %Take the correct matrices thanks to the direction estimated with the
    %discrete classifier
    A=Dir(1,ktest).A;
    H=Dir(1,ktest).H;
    W=Dir(1,ktest).W;
    Q=Dir(1,ktest).Q;

    %Initialization of the estimated vector xhat with the real initial
    %position.
    %xhat=zeros(DimStateVec,MaxTimeTest-StartingTime);
        
    if (t==20),
        xhat=zeros(6,1);
        
        x0(1)=past_trials(ntest).startHandPos(1);
        x0(2)=past_trials(ntest).startHandPos(2);
        x0(3)=0;
        x0(4)=0;
        x0(5)=0;
        x0(6)=0;
        
    else
        xhat=[past_trials(ntest).decodedHandPos zeros(6,1)];
    end
    
    if (t==20),
        Pe=W; % pm1=0;
        Dir(1,ktest).P=zeros(DimStateVec,DimStateVec);
    end
    %Load Pm1
    Pm1=Dir(1,ktest).P;
   
    %Kalman filter
    %the goal is to find an equation that computes an a
    %posteriori state estimate xhat(k) as a linear combination of a priori
    %estimate xhat(-) and a weighted difference between an actual
    %measurement zk and a measurement prediction H*xhat(-)
    % xhat=xhat(-)+Kk(zk-H*xhat(-))
    
      
    zk=zeros(98,1);
    if t==20,
        for i=1:20,
            zk=zk+past_trials(ntest).spikes(:,StartingTime+i);
        end
    else
        for i=1:20,
            zk=zk+past_trials(ntest).spikes(:,(t-20)+StartingTime+i);
        end
    end

    
    %Computing the state estimate a priori
    if t==20,
        xhat(:,1)=A*x0';
    else
        xhat(:,end)=A*xhat(:,end-1);
    end
    if t>20,
        Pe=A*Pm1*A'+W; 
    end
        Kk=Pe*(H')*pinv(H*Pe*H'+Q); %Kalman filter gain
        %Compute the state estimate a posteriori in function of the a
        %priori one 
        xhat(:,end)=xhat(:,end)+Kk*(zk-H*xhat(:,end));
        P=(eye(DimStateVec)-Kk*H)*Pe;
      
% 
    if ktest==1,
        if xhat(1,end)>78,
            xhat(1,end)=78;
        end
    elseif ktest==2,
        if xhat(2,end)>94,
            xhat(2,end)=94;
        end
    elseif ktest==3,
        if xhat(2,end)>98,
            xhat(2,end)=98;
        end    
    elseif ktest==4,
        if xhat(1,end)<-101,
            xhat(1,end)=-101;
        end  
    elseif ktest==5,
        if xhat(1,end)<-109,
            xhat(1,end)=-109;
        end   
    elseif ktest==6,
        if xhat(1,end)<-78,
            xhat(1,end)=-78;
        end
        if xhat(2,end)<-85,
            xhat(2,end)=-85;
        end   
    elseif ktest==7,
        if xhat(2,end)<-85,
            xhat(2,end)=-85;
        end  
    elseif ktest==8,
        if xhat(1,end)>92,
            xhat(1,end)=94;
        end        
    end    

% 

    Dir(1,ktest).P=P; 
    decodedPos=xhat(:,end)';
    newpar=Dir;

end