function [ to_assign ] = classifier_k( mat_all, data_to_class )
% data_to_class : 1 trial !
vectg=[18 27 31 44 69 81 84 90 93 97];
k=30;
firing_rate = struct;
spike=data_to_class.spikes(vectg,201:end);
firing_rate_to_class = sum(spike,2);%firing rate for each neuron Nbr x 1 righe

for tri = 1:1:size(mat_all,1) % nbr trials
    for ang=1:1:size(mat_all,2) % nbr angles
        firing_rate(tri,ang).fire = sum(mat_all(tri, ang).spikes(:,:),2); % vector de la taille neurons Nbr (22) x 1
        firing_dist(tri,ang) = sum((firing_rate_to_class-firing_rate(tri,ang).fire).^2);
    end
end


%find k nearest neighbours
max_fire = max(max(firing_dist));
var = zeros(1,size(mat_all,2));
for j =1:1:k
    
    [mina Ia] = min(firing_dist);
    [minb Ib] = min(mina);
    
    column_min = Ib; % i.e. the angle !
    line_min = Ia(Ib);
    
    firing_dist(line_min,column_min) = max_fire;
    angles_nearest(j) = column_min;
    var(column_min)=var(column_min)+1;
end


[maxim ind] = max(var); % index : angle to assign

index= [];
for i = 1:1:8 % check for maximum (only 1 angle or many->tie)
    if var(i) == maxim
        index = [index i];
    end
end

if length(index)>1 % i.e. there is a tie
    %mean of distances smallest
    [meana iame] = min(mean(firing_dist(:,index)));
    to_assign = index(iame(1));
else
    to_assign = ind;
end


end
