function val = DerLamda(lamda, tspan)%tf)
    %t_arr = linspace(0, tf, length(lamda));
    t_step = tspan(2) - tspan(1);
    range = 1:length(tspan);
    val = 0*lamda;
    
    for j=range
        if j == length(range)
            val(j,:) = lamda(j,:);%val(j-1);
        else
            val(j,:) = (lamda(j+1,:)-lamda(j,:))/t_step;
        end
    end
end