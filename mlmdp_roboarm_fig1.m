%%
% Multitask Z-learning framework
% Andrew Saxe
% 5/13/16
%%

clear all


%% Set up grid world.

N = 50;

[X,Y] = meshgrid(1:N);
xy = [X(:) Y(:)];

P = zeros(N*N,N*N);
ind = 1;
for i = 1:N
    for j = 1:N
              
        mu = [i j];
        sigma = eye(2)*1;

        p = mvnpdf(tmp, mu, sigma);
       
        
        P(:, ind) = p(:);
        ind = ind + 1;
        
        

    end
end
P(P<.05)=0;
P = sparse(P);
P = bsxfun(@rdivide,P,sum(P));
figure()
imagesc(P); colormap jet
title('Passive dynamics matrix')


% Add boundary transition probabilities
alpha = .000001;
P = (1-alpha)*P;
Pib = alpha*speye(N*N);


%% Set up subtask reward structures (for boundary states)

clear r

lambda = .1;
Nr = N*N;


sigma = 1;

[X,Y]=meshgrid(1:N,1:N);
i=1;
for x = 1:1:N
    for y = 1:1:N
        r(:,i) = reshape(-((X-x).^2 +(Y-y).^2)/(2*sigma.^2),Nr,1);
        i=i+1;
    end
end
Nt = size(r,2);


ri = -.1*ones(Nr,1);
figure()
imagesc(reshape(r(:,2),N,N)); colorbar
title('immediate reward')

%% Compute optimal cost-to-go for all subtasks

q = exp(r/lambda);
qi = exp(ri/lambda);

zexact = zeros(size(q));
zbexact = q;
Nitrs = round(Nr*1.5);
for i = 1:Nt
    i
    M = sparse(diag(qi)*P);
    Nzb = sparse(diag(qi)*Pib*zbexact(:,i));
     %for j = 1:Nitrs
     %    zexact(:,i) = M*zexact(:,i) + Nzb;
     %end
    zexact(:,i) = (speye(size(M))-M)\Nzb;
    
    should_plot = false;
    if should_plot
        subplot(211)
        imagesc(reshape(r(:,i),N,N)); colormap jet; colorbar
  
        title('r')
        subplot(212)
        imagesc(reshape(log(zexact(:,i)),N,N)); colorbar
        title('Optimal cost-to-go')
        drawnow
    end
    
    
end



%% Roboarm
N = 50;
theta1 = linspace(-pi,pi,N);
theta2 = linspace(-pi,pi,N);

[th1,th2] = meshgrid(theta1,theta2);

thv1 = th1(:);
thv2 = th2(:);
L = 1;
xs = L*cos(thv1);
ys = L*sin(thv1);

xg = xs - L*cos(pi-thv2-thv1);
yg = ys + L*sin(pi-thv2-thv1);

s = 106;
thv1(s)*180/pi
thv2(s)*180/pi
figure()
plot([0 xs(s) xg(s)],[0 ys(s) yg(s)],'linewidth',2)
xlim([-2 2])
ylim([0 2])
plot(xg,yg,'.')

% Find end effector points along a line

goal_line = abs(yg - 1)<.1 & abs(xg-1)<30;

imagesc(reshape(goal_line,N,N))
title('Goal states')

%% Plot all goal configs
figure()
inds = find(goal_line==1);
rectangle('Position',[-2 .9 4 .2],'FaceColor','k','EdgeColor','none')
hold on
for i = 1:length(inds)
    plot([0 xs(inds(i)) xg(inds(i))],[0 ys(inds(i)) yg(inds(i))],'b','linewidth',.5)
    hold on
end
scatter(xg(goal_line),yg(goal_line),70,[1 0 0],'filled')
xlim([-2 2])
ylim([-.5 2])
axis equal
axis off

%% Plot example arm position
%inds = find(goal_line==1);
figure()
i = 1640;
%rectangle('Position',[-2 .9 4 .2],'FaceColor','k','EdgeColor','none')
hold on
plot([0 xs(i) xg(i)],[0 ys(i) yg(i)],'b','linewidth',3)
scatter(xg(i),yg(i),70,[1 0 0],'filled')
xlim([-2 2])
ylim([-.5 2])
axis off
axis equal

%% Set up actual task reward structure

qg = exp(goal_line/lambda);


w = pinv(q)*qg;
w(w<0)=0;
qest = q*w;




%% Plot MLMDP solution

w(w<0)=0;
w(w<1e4)=0;
qrew = q*w;
zact = zexact*w;
zactb = zbexact*w;

rrew = lambda*log(q*w);


% Compute controlled transition mtx U
U = sparse(diag(zact)*P);
Ub = sparse(diag(zactb)*Pib);
normaliz = sum([U; Ub]);%
U = bsxfun(@rdivide,U,normaliz);
Ub = bsxfun(@rdivide,Ub,normaliz);
    

% Plot trajectories

starting_pos = [ 4 19; 20 40;  30 20];%;45 12;16 40;   9 20;13 24 ;
colors = .7*[.3 .8 .8; .8 .8 .3; .8 .3 .8; .4 .4 .4];

figure()
plot_roboarm_trajectories(U,starting_pos,colors,xs,ys,xg,yg,xy,zact,rrew)
title('Compositional MLMDP solution')

%% Plot component trajectories

wc = zeros(size(w));

wc(125) = 1; % Change this index to generate different component trajectories

qrewc = q*wc;
zactc = zexact*wc;
zactbc = zbexact*wc;

rrewc = lambda*log(q*wc);

U = sparse(diag(zactc)*P);
Ub = sparse(diag(zactbc)*Pib);
normaliz = sum([U; Ub]);%
U = bsxfun(@rdivide,U,normaliz);
Ub = bsxfun(@rdivide,Ub,normaliz);

starting_pos = [ 4 19; 20 40;  30 20];%;45 12;16 40;   9 20;13 24 ;
colors = .7*[.3 .8 .8; .8 .8 .3; .8 .3 .8; .4 .4 .4];

figure()

plot_roboarm_trajectories(U,starting_pos,colors,xs,ys,xg,yg,xy,zactc,rrewc)
title('Component MLMDP solution')

%% Plot actual optimal solution
lambda = .1;
rtmp = goal_line*10.1-.1;
qg = exp(rtmp/lambda);
qi = exp(ri/lambda);
M = sparse(diag(qi)*P);
Nzb = sparse(diag(qi)*Pib*qg);


zg = (speye(size(M))-M)\Nzb;
Nitrs = 1000;
for j = 1:Nitrs
    zg = M*zg + Nzb;
end

U = sparse(diag(zg)*P);
Ub = sparse(diag(qg)*Pib);
normaliz = sum([U; Ub]);%
U = bsxfun(@rdivide,U,normaliz);
Ub = bsxfun(@rdivide,Ub,normaliz);

starting_pos = [ 4 19; 20 40;  30 20];%;45 12;16 40;   9 20;13 24 ;
colors = .7*[.3 .8 .8; .8 .8 .3; .8 .3 .8; .4 .4 .4];


figure()
plot_roboarm_trajectories(U,starting_pos,colors,xs,ys,xg,yg,xy,zg,log(qg)*lambda)

title('Direct solution')




