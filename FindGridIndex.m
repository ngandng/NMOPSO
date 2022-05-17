%
% Copyright (c) 2015, Mostapha Kalami Heris & Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "LICENSE" file for license terms.
%
% Project Code: YPEA121
% Project Title: Multi-Objective Particle Swarm Optimization (MOPSO)
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Cite as:
% Mostapha Kalami Heris, Multi-Objective PSO in MATLAB (URL: https://yarpiz.com/59/ypea121-mopso), Yarpiz, 2015.
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function particle = FindGridIndex(particle, Grid)

    nObj = numel(particle.Cost);
    
    nGrid = numel(Grid(1).LB);
    
    particle.GridSubIndex = zeros(1, nObj);
    idx = zeros(1, nObj);
    
    for j = 1:nObj
        
        if isempty(find(particle.Cost(j)<Grid(j).UB, 1, 'first'))
            idx(j) = nGrid;
        else
            idx(j) = find(particle.Cost(j)<Grid(j).UB, 1, 'first'); % 1 so dau tien thoa man...
        end
    end
    particle.GridSubIndex = idx;

    particle.GridIndex = particle.GridSubIndex(1);
    for j = 2:nObj
        particle.GridIndex = particle.GridIndex-1;
        particle.GridIndex = nGrid*particle.GridIndex;
        particle.GridIndex = particle.GridIndex+particle.GridSubIndex(j);
    end
    
end