
function Grid = CreateGrid(pop, nGrid, alpha)

    c = [pop.Cost]; 
    
    cmin = min(c, [], 2); 
    cmax = max(c, [], 2);
    
    dc = cmax-cmin;
    cmin = cmin-alpha*dc;
    cmax = cmax+alpha*dc;
    
    nObj = size(c, 1); 
    
    empty_grid.LB = [];
    empty_grid.UB = [];
    Grid = repmat(empty_grid, nObj, 1);
    
    for j = 1:nObj
        
        cj = linspace(cmin(j), cmax(j), nGrid+1); 
        
        Grid(j).LB = [-inf cj];
        Grid(j).UB = [cj +inf];
        
    end

end