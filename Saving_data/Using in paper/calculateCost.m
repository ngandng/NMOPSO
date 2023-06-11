clear; close all;

addpath('C:\Users\nganu\OneDrive\Documents\02. Nghiên cứu\UAV Path planning\Comparision\Model');

rep = load('MOSPSO_Rep5.mat');
rep = rep.rep;

model = CreateModel5();

for i=1:numel(rep)
    disp(i);
    rep(i).Position = SphericalToCart2(rep(i).Position, model);
    rep(i).Cost = MyCost(rep(i).Position, model);
end
