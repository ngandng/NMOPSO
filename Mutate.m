function xnew = Mutate(x,pm,delta,VarMax,VarMin)

    nVar = numel(x.Position.r);
    pbest = x.Best;
    
    beta = tanh(delta*length(pm)); % alpha/F in reference

%     if(rand<beta)
        
        xnew.r = x.Position.r + randn(1,nVar).*pbest.Position.r*beta;
        xnew.phi = x.Position.phi + randn(1,nVar).*pbest.Position.phi*beta;
        xnew.psi = x.Position.psi + randn(1,nVar).*pbest.Position.psi*beta;

        xnew.r = max(VarMax.r,xnew.r);
        xnew.r = min(VarMin.r,xnew.r);

        xnew.phi = max(VarMax.phi,xnew.phi);
        xnew.phi = min(VarMin.phi,xnew.phi);

        xnew.psi = max(VarMax.psi,xnew.psi);
        xnew.psi = min(VarMin.psi,xnew.psi);
    
% %     else
%         xnew.r = x.Position.r;
%         xnew.phi = x.Position.phi;
%         xnew.psi = x.Position.psi;
%     end

end