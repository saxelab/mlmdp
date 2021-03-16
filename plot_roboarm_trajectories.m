function plot_roboarm_trajectories(U,starting_pos,colors,xs,ys,xg,yg,xy,zact,rrew)

N = sqrt(size(U,2));

figure(1)
subplot(312)
scatter(xy(:,1),xy(:,2),10,exp(rrew),'fill','s')%U(:,state),'fill')
hold on
axis equal
axis off

subplot(313)

scatter(xy(:,1),xy(:,2),10,log(zact),'fill','s')%U(:,state),'fill')
hold on



for sp = 1:size(starting_pos,1)
    grid = zeros(N,N);
    grid(starting_pos(sp,1),starting_pos(sp,2)) = 1;
    
    statehist = compute_maximal_trajectory(U,find(grid(:)==1));
    
    
    subplot(313)
    plot(xy(statehist,1),xy(statehist,2),'Color',colors(sp,:),'linewidth',2,'MarkerSize',25);
    hold on
    scatter(xy(statehist(1),1), xy(statehist(1),2),70,[0 1 0],'fill')
    hold on
     scatter(xy(statehist(end),1), xy(statehist(end),2),70,[1 0 0],'fill')
    hold on
    axis equal
    axis off
   

    % Plot as strobed arm trajectory

    draw_arm = true;
    if draw_arm
        subplot(311)
        if sp == 1
            rectangle('Position',[-2 .9 4 .2],'FaceColor','k','EdgeColor','none')
            hold on
        end
        for i = 1:length(statehist)
            if i == 1
                c = [0 1 0];
            elseif i == length(statehist)
                c = [1 0 0];
            else
               c = colors(sp,:); 
            end

            plot([0 xs(statehist(i)) xg(statehist(i))],[0 ys(statehist(i)) yg(statehist(i))],'Color',colors(sp,:),'linewidth',.5)
            if  i == length(statehist)
                scatter(xg(statehist(i)), yg(statehist(i)),70,c,'fill')
            end
            hold on
        end
      
        scatter(xg(statehist(1)), yg(statehist(1)),70,[0 1 0],'fill')
       
        
        %plot(xg(goal_line),yg(goal_line),'ro','MarkerSize',5)
        xlim([-2.5 2.5])
        ylim([-2 2])
        axis equal
        axis off
   end
end