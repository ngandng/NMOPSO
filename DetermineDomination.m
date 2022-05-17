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

function pop = DetermineDomination(pop)

    nPop = numel(pop);
    
    for i = 1:nPop
        pop(i).IsDominated = false;
    end
    
    for i = 1:nPop-1
        for j = i+1:nPop
            
            if Dominates(pop(i), pop(j))
               pop(j).IsDominated = true;
            end
            
            if Dominates(pop(j), pop(i))
               pop(i).IsDominated = true;
            end
            
        end
    end
    
    for k = 1:nPop
        if any(pop(k).Cost>=inf)
            pop(k).IsDominated = true;
        end
    end

end