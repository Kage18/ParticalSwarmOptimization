
clc;
clear;
close all;

%% Problem Definition

CostFunction=@(x,i,nPop) cost(x,i,nPop);        % Cost Function

nVar=2;            % Number of Decision Variables

VarSize=[1 nVar];   % Size of Decision Variables Matrix

VarMin=-100;         % Lower Bound of Variables
VarMax= 100;         % Upper Bound of Variables


%% PSO Parameters

MaxIt=250;      % Maximum Number of Iterations

nPop=50;        % Population Size (Swarm Size)

% PSO Parameters
% w=1;            % Inertia Weight
% wdamp=0.99;     % Inertia Weight Damping Ratio
% c1=1.5;         % Personal Learning Coefficient
% c2=2.0;         % Global Learning Coefficient

% If you would like to use Constriction Coefficients for PSO,
% uncomment the following block and comment the above set of parameters.

% % Constriction Coefficients
phi1=2.05;
phi2=2.05;
phi=phi1+phi2;
chi=2/(phi-2+sqrt(phi^2-4*phi));
w=chi;          % Inertia Weight
wdamp=1;        % Inertia Weight Damping Ratio
c1=chi*phi1;    % Personal Learning Coefficient
c2=chi*phi2;    % Global Learning Coefficient

% Velocity Limits
VelMax=0.004*(VarMax-VarMin);
VelMin=-VelMax;

%% Initialization

empty_particle.Position=[];
empty_particle.Cost=[];
empty_particle.Velocity=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
empty_particle.Energy=[];

particle=repmat(empty_particle,nPop,1);
distancecost=inf;
GlobalBest.Cost=inf;
GlobalBestindex=1;
rand('state', 0);
for i=1:nPop
    
    % Initialize Position
    particle(i).Position=unifrnd(VarMin,VarMax,VarSize);
    
    % Initialize Velocity
    particle(i).Velocity=zeros(VarSize);
    
    %Energy
    particle(i).Energy = 5;
    % Evaluation
%     particle(i).Cost = CostFunction(particle,i,nPop);
%     
    % Update Personal Best
    particle(i).Best.Position = particle(i).Position;

    
end

for i = 1:nPop
    
    [particle(i).Cost,distance] = CostFunction(particle,i,nPop);
    particle(i).Best.Cost = particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest=particle(i).Best;
        GlobalBestIndex=i;
        
    end

end
BestCost=zeros(MaxIt,1);

%% PSO Main Loop

for it=1:MaxIt
    
    clf
    distancecost=0;
    dead=0;
    for i=1:nPop
        if particle(i).Energy>=.05
            % Update Velocity
            particle(i).Velocity = w*particle(i).Velocity ...
                +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
                +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);

            % Apply Velocity Limits
            particle(i).Velocity = max(particle(i).Velocity,VelMin);
            particle(i).Velocity = min(particle(i).Velocity,VelMax);

            % Update Position
            particle(i).Position = particle(i).Position + particle(i).Velocity;

            % Velocity Mirror Effect
            IsOutside=(particle(i).Position<VarMin | particle(i).Position>VarMax);
            particle(i).Velocity(IsOutside)=-particle(i).Velocity(IsOutside);

            % Apply Position Limits
            particle(i).Position = max(particle(i).Position,VarMin);
            particle(i).Position = min(particle(i).Position,VarMax);

            % Evaluation
            particle(i).Cost = CostFunction(particle,i,nPop);

            % Update Personal Best
            if particle(i).Cost<particle(i).Best.Cost

                particle(i).Best.Position=particle(i).Position;
                particle(i).Best.Cost=particle(i).Cost;

                % Update Global Best
                if particle(i).Best.Cost<GlobalBest.Cost

                    GlobalBest=particle(i).Best;
                    GlobalBestIndex=i;
                end
            end
%             if distance<distancecost
%         
%                 distancecost=distance;
%         
%             end
            distancecost=distancecost+(distance)/nPop;
        end
        
        
    if particle(i).Energy<.05
        plot(particle(i).Position(1),particle(i).Position(2),'rx');hold on;
    else
        plot(particle(i).Position(1),particle(i).Position(2),'bo');hold on;
    end
    if i==nPop
        plot(GlobalBest.Position(1),GlobalBest.Position(2),'go');hold on;
    end
    axis([-100 100 -100 100]);
    end

    pause(.1);
    for i= 1:nPop
       particle(i).Energy = particle(i).Energy-((((4*pi*((particle(GlobalBestIndex).Position(1) - particle(i).Position(1)).^2 + (particle(GlobalBestIndex).Position(2) - particle(i).Position(2)).^2))/0.125)^2)*(10^-4))/(2.4*(10^9));
    end
    
    for j = 1:nPop
    
    particle(GlobalBestIndex).Energy = particle(GlobalBestIndex).Energy -((((4*pi*((particle(GlobalBestIndex).Position(1) - particle(j).Position(1)).^2 + (particle(GlobalBestIndex).Position(2) - particle(j).Position(2)).^2))/0.125)^2)*(10^-4))/(2.4*(10^9));
    end
    BestCost(it)=distancecost;
  
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    w=w*wdamp;
    
end

BestSol = GlobalBest;

%% Results
deadnodes = 0;
for i= 1:nPop
    if particle(i).Energy < .05
       deadnodes = deadnodes + 1;
    end
end
figure;
semilogy(BestCost,'LineWidth',2);
title(['At the cost of ' num2str(deadnodes) ' nodes']);
xlabel('Iteration');
ylabel('Best Cost');
% figure;
% plot(BestCost,'LineWidth',2);
% title(['At the cost of ' num2str(deadnodes) ' nodes']);
% xlabel('Iteration');
% ylabel('Best Cost');
grid on;
