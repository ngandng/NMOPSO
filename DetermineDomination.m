
function pop = DetermineDomination(pop)

    nPop = numel(pop);
    
    for i = 1:nPop
        pop(i).IsDominated = false;
    end
    
    for i = 1:nPop-1
        
        if any(pop(i).Cost == Inf)
            pop(i).IsDominated = true;
        end
        
        for j = i+1:nPop
            
            if Dominates(pop(i), pop(j))
               pop(j).IsDominated = true;
            end
            
            if Dominates(pop(j), pop(i))
               pop(i).IsDominated = true;
            end
            
        end
        
    end
    
    if any(pop(nPop).Cost == Inf)
            pop(nPop).IsDominated = true;
    end

end