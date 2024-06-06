
function particle = FindGridIndex(particle, Grid)

    nObj = numel(particle.Cost);
    
    nGrid = numel(Grid(1).LB)-2;
    
    particle.GridSubIndex = zeros(1, nObj);
    idx = zeros(1, nObj);
    
    for j = 1:nObj
        
        if isempty(find(particle.Cost(j)<Grid(j).UB, 1, 'first'))
            idx(j) = nGrid;
        else
            idx(j) = find(particle.Cost(j)<Grid(j).UB, 1, 'first'); 
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