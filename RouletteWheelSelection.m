
function i = RouletteWheelSelection(P)

    r = rand;
    
    C = cumsum(P); % cumulative sum
    
    i = find(r <= C, 1, 'first');

end