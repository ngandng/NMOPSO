
function pop = DetermineDomination(pop)

    nPop = numel(pop);
    
    for i = 1:nPop
        pop(i).IsDominated = false;
    end
    
    % for the pop 2:nPop-1
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
    % for the end pop
    if any(pop(end).Cost == Inf)
            pop(end).IsDominated = true;
    end

    for j = 1:nPop-1
        
        if Dominates(pop(end), pop(j))
           pop(j).IsDominated = true;
        end
        
        if Dominates(pop(j), pop(end))
           pop(end).IsDominated = true;
           break;
        end 
    end

end