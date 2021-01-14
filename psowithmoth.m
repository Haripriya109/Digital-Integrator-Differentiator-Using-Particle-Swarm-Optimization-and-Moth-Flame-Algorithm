clc;                         
clear;
close all;

%% Problem definition

              
nVar = 5;                                    %no. of unknown variables
VarSize = [1 nVar];                          %matrix size of variables 
VarMin = -1;                                %minimum value of variables
VarMax = 1;                                 %maximum value of variables

%% Parameters of PSO

MaxIt = 1;                                 %maximum no. of iterations
nPop = 50;                                   %population/swarm size
w = 1;                                       %inertia coefficient
wdamp = 0.99;                                %damping ratio of inertia coefficient
c1 = 2;                                      %personal acceleration coefficient 
c2 = 2;                                       %social acceleration coefficient 

%%Initialization


empty_particle.Position = [];                %position of particle
empty_particle.Velocity = [];                %velocity of particle
empty_particle.Cost = [];                    %particle's own measurement
empty_particle.Best.Position = [];           %own best value
empty_particle.Best.Cost = [];               %own best cost

particle = repmat(empty_particle, nPop, 1);  %repeatMatrix=> to use above structures for all 50 particles
GlobalBest.Cost = inf;                       %worsst value for comparison

for i=1:nPop
    [OBFS BFS]=MFOAlgo();
    particle(i).Position = BFS(1,:); %generating random numbers
    particle(i).Velocity = zeros(VarSize);
  
 
    particle(i).Cost = minor1(particle(i).Position);
   
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    
    if particle(i).Best.Cost < GlobalBest.Cost
       GlobalBest.Cost = particle(i).Best.Cost;
       GlobalBest.Position= particle(i).Best.Position;
   end
end
BestCosts = zeros(MaxIt, 1);        %array to hold best cost value on each iteration

%% Main loop of PSO

for it=1:MaxIt
    
    for i=1:nPop
        %updating velocity
        particle(i).Velocity = w*particle(i).Velocity + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);
        particle(i).Position = particle(i).Position + particle(i).Velocity;     %updating position 
        particle(i).Cost = minor1(particle(i).Position);                  
        
        if particle(i).Cost < particle(i).Best.Cost                             %update personal best
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            
            if particle(i).Best.Cost < GlobalBest.Cost                          %update global best
                GlobalBest.Cost = particle(i).Best.Cost;
                GlobalBest.Position= particle(i).Best.Position;
                
            end
        end
    end
    BestCosts(it)=GlobalBest.Cost;
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);      %displaying iteration information
    w = w*wdamp;   %damping    
end


        