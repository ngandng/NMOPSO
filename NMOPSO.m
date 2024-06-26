clc; close; clear all;
%% Problem Definition
model = CreateModel(); % Create search map and parameters
model_name = 6;

nVar=model.n;       % Number of Decision Variables = searching dimension of PSO = number of path nodes

VarSize=[1 nVar];   % Size of Decision Variables Matrix

% Lower and upper Bounds of particles (Variables)
VarMin.x=model.xmin;           
VarMax.x=model.xmax;           
VarMin.y=model.ymin;           
VarMax.y=model.ymax;           
VarMin.z=model.zmin;           
VarMax.z=model.zmax;                 

VarMax.r=3*norm(model.start-model.end)/nVar;  % r is distance
VarMin.r=VarMax.r/9;

% Inclination (elevation)
AngleRange = pi/4; % Limit the angle range for better solutions
VarMin.psi=-AngleRange;            
VarMax.psi=AngleRange;          

% Azimuth 
VarMin.phi=-AngleRange;            
VarMax.phi=AngleRange;          

% Lower and upper Bounds of 
alpha=0.5;
VelMax.r=alpha*(VarMax.r-VarMin.r);    
VelMin.r=-VelMax.r;                    
VelMax.psi=alpha*(VarMax.psi-VarMin.psi);    
VelMin.psi=-VelMax.psi;                    
VelMax.phi=alpha*(VarMax.phi-VarMin.phi);    
VelMin.phi=-VelMax.phi;   

CostFunction=@(x) MyCost(x,model,VarMin);    % Cost Function

%% PSO Parameters

dummy_output = CostFunction(struct('x', ones(1, model.n), 'y', ones(1, model.n), 'z', ones(1, model.n)));
nObj = numel(dummy_output);                   % Determine the number of objectives

MaxIt = 500;          % Maximum Number of Iterations

nPop = 100;           % Population Size (Swarm Size)
        
nRep = 50;            % Repository Size

w = 1;                % Inertia Weight
wdamp = 0.98;         % Inertia Weight Damping Ratio
c1 = 1.5;             % Personal Learning Coefficient
c2 = 1.5;             % Global Learning Coefficient

nGrid = 5;            % Number of Grids per Dimension
alpha = 0.1;          % Inflation Rate

beta = 2;             % Leader Selection Pressure
gamma = 2;            % Deletion Selection Pressure

mu = 0.5;             % Mutation Rate
delta = 20;           % delta = num(rep)/10

%% Initialization
% Create Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];
empty_particle.IsDominated = [];
empty_particle.GridIndex = [];
empty_particle.GridSubIndex = [];

% Initialize Global Best
GlobalBest.Cost=Inf(nObj,1); % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);

isInit = false;
% tic;
while (~isInit)
    disp('Initialising...');
    for i=1:nPop

        % Initialize Position
        particle(i).Position=CreateRandomSolution(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle(i).Velocity.r=zeros(VarSize);
        particle(i).Velocity.psi=zeros(VarSize);
        particle(i).Velocity.phi=zeros(VarSize);

        % Evaluation
        particle(i).Cost= CostFunction(SphericalToCart2(particle(i).Position,model));

        % Update Personal Best
        particle(i).Best.Position=particle(i).Position;
        particle(i).Best.Cost=particle(i).Cost;

        % Update Global Best
        if Dominates(particle(i).Best.Cost,GlobalBest.Cost)
            GlobalBest=particle(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost=zeros(MaxIt,nObj);

% Determine Domination
particle = DetermineDomination(particle);

rep = particle(~[particle.IsDominated]); % the un-dominated 
% rep.GridSubIndex = [];
% rep.GridIndex = [];

Grid = CreateGrid(rep, nGrid, alpha);

for i = 1:numel(rep)
    rep(i) = FindGridIndex(rep(i), Grid);
end

%% PSO Main Loop

for it=1:(MaxIt)

    % Update Best Cost Ever Found
    BestCost(it,:)=GlobalBest.Cost;
    
    for i=1:nPop   
        
        % select leader = update global best
        GlobalBest = SelectLeader(rep, beta);
        
        
        % ----------------------r Part--------------------------        
        % Update Velocity
        particle(i).Velocity.r = w*particle(i).Velocity.r ...
            + c1*rand(VarSize).*(particle(i).Best.Position.r-particle(i).Position.r) ...
            + c2*rand(VarSize).*(GlobalBest.Position.r-particle(i).Position.r);

        % Update Velocity Bounds
        particle(i).Velocity.r = max(particle(i).Velocity.r,VelMin.r);
        particle(i).Velocity.r = min(particle(i).Velocity.r,VelMax.r);

        % Update Position
        particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle(i).Position.r<VarMin.r | particle(i).Position.r>VarMax.r);
        particle(i).Velocity.r(OutOfTheRange)=-particle(i).Velocity.r(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.r = max(particle(i).Position.r,VarMin.r);
        particle(i).Position.r = min(particle(i).Position.r,VarMax.r);

        
        % -------------------psi Part----------------------

        % Update Velocity
        particle(i).Velocity.psi = w*particle(i).Velocity.psi ...
            + c1*rand(VarSize).*(particle(i).Best.Position.psi-particle(i).Position.psi) ...
            + c2*rand(VarSize).*(GlobalBest.Position.psi-particle(i).Position.psi);

        % Update Velocity Bounds
        particle(i).Velocity.psi = max(particle(i).Velocity.psi,VelMin.psi);
        particle(i).Velocity.psi = min(particle(i).Velocity.psi,VelMax.psi);

        % Update Position
        particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.psi<VarMin.psi | particle(i).Position.psi>VarMax.psi);
        particle(i).Velocity.psi(OutOfTheRange)=-particle(i).Velocity.psi(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.psi = max(particle(i).Position.psi,VarMin.psi);
        particle(i).Position.psi = min(particle(i).Position.psi,VarMax.psi);

        
        % -----------------------Phi part----------------------------------
        % Update Velocity
        particle(i).Velocity.phi = w*particle(i).Velocity.phi ...
            + c1*rand(VarSize).*(particle(i).Best.Position.phi-particle(i).Position.phi) ...
            + c2*rand(VarSize).*(GlobalBest.Position.phi-particle(i).Position.phi);

        % Update Velocity Bounds
        particle(i).Velocity.phi = max(particle(i).Velocity.phi,VelMin.phi);
        particle(i).Velocity.phi = min(particle(i).Velocity.phi,VelMax.phi);

        % Update Position
        particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.phi<VarMin.phi | particle(i).Position.phi>VarMax.phi);
        particle(i).Velocity.phi(OutOfTheRange)=-particle(i).Velocity.phi(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.phi = max(particle(i).Position.phi,VarMin.phi);
        particle(i).Position.phi = min(particle(i).Position.phi,VarMax.phi);

                
        %----------------------- Evaluation------------------------
        particle(i).Cost=CostFunction(SphericalToCart2(particle(i).Position,model));
        
        %------ Apply mutation-------
        pm = (1-(it-1)/(MaxIt-1))^(1/mu);
        if rand<pm
            NewSol.Position = Mutate(particle(i),rep,delta,VarMax,VarMin);
            NewSol.Cost = CostFunction(SphericalToCart2(NewSol.Position,model));
            if Dominates(NewSol, particle(i))
                particle(i).Position = NewSol.Position;
                particle(i).Cost = NewSol.Cost;

            elseif Dominates(particle(i),NewSol)
                %do nothing

            else
                if rand < 0.5
                    particle(i).Position = NewSol.Position;
                    particle(i).Cost = NewSol.Cost;
                end
            end
        end
        % Update Personal Best
            
        if Dominates(particle(i), particle(i).Best)

            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            
        elseif Dominates(particle(i).Best, particle(i))
            % Do Nothing
            
        else
            if rand<0.5
                particle(i).Best.Position = particle(i).Position;
                particle(i).Best.Cost = particle(i).Cost;
            end
        end

    end
    
    % Determine Domination
    particle = DetermineDomination(particle);

    % Add Non-Dominated Particles to REPOSITORY
    rep = [rep
         particle(~[particle.IsDominated])]; %#ok
    
    % Determine Domination of New Resository Members
    rep = DetermineDomination(rep);
    
    % Keep only Non-Dminated Members in the Repository
    rep = rep(~[rep.IsDominated]);
    
    % Update Grid
    Grid = CreateGrid(rep, nGrid, alpha);
    
    % Update Grid Indices
    for i = 1:numel(rep)
        rep(i) = FindGridIndex(rep(i), Grid);
    end
    
    % Check if Repository is Full
    if numel(rep)>nRep
        Extra = numel(rep)-nRep;
        for e = 1:Extra
            rep = DeleteOneRepMember(rep, gamma);
        end   
    end
    
    % Inertia Weight Damping
    w=w*wdamp;

%     Show Iteration Information 
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it,:))]);
end

GlobalBest = SelectLeader(rep, beta);

%% Plot results
% Best solution
BestPosition = SphericalToCart2(GlobalBest.Position,model);
smooth = 1;
PlotSolution(BestPosition,model,smooth);

