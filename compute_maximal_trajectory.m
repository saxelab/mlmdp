function states = compute_maximal_trajectory(U,state0)
state0
ind = 2;
states(1)=state0;
while ind < 50 %isempty(intersect(state, find(w>0)))

    tmp = U(:,states(ind-1));
    %tmp(states(ind-1)) = 0;

    [m,states(ind)] = max(tmp);
    ind = ind + 1;
end