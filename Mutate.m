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

function xnew = Mutate(x, pm, VarMin, VarMax)

    nVar = numel(x.r);
    j = randi([1 nVar]);

    dx.r = pm*(VarMax.r-VarMin.r);
    dx.phi = pm*(VarMax.phi-VarMin.phi);
    dx.psi = pm*(VarMax.psi-VarMin.psi);
    
    lb.r = x.r(j) - dx.r;
    lb.r = max(lb.r,VarMin.r);
    ub.r = x.r(j) + dx.r;
    ub.r = min(ub.r,VarMax.r);
    xnew.r = x.r;
    xnew.r(j) = unifrnd(lb.r, ub.r);
    
    lb.psi = x.psi(j) - dx.psi;
    lb.psi = max(lb.psi,VarMin.psi);
    ub.psi = x.psi(j) + dx.psi;
    ub.psi = min(ub.psi,VarMax.psi);
    xnew.psi = x.psi;
    xnew.psi(j) = unifrnd(lb.psi, ub.psi);
    
    lb.phi = x.phi(j) - dx.phi;
    lb.phi = max(lb.phi,VarMin.phi);
    ub.phi = x.phi(j) + dx.phi;
    ub.phi = min(ub.phi,VarMax.phi);
    xnew.phi = x.phi;
    xnew.phi(j) = unifrnd(lb.phi, ub.phi);

end