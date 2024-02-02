% the positive direction is counterclockwise

function position = SphericalToCart2(solution,model)
%% solution of sphere space
r = solution.r;
phi = solution.phi;
psi = solution.psi;

%% find the start matrix
%start position is 4*4 matrix that including postion and orientation
xs = model.start(1);
ys = model.start(2);
zs = model.start(3);
%supplement the start orientation
start = [1, 0, 0, xs;...
         0, 1, 0, ys;...
         0, 0, 1, zs;...
         0, 0, 0,  1];
dirVector = model.end - model.start;

phistart = atan2(dirVector(2),dirVector(1));
psistart = atan2(dirVector(3),norm([dirVector(1),dirVector(2)]));

dir = TransfomationMatrix(0,phistart,psistart);
startPosition = start*dir;

%% find the position of each particle
% T is the transformation matrix from start position to i position
T(1).value = TransfomationMatrix(r(1),phi(1),psi(1));
pos(1).value = startPosition*T(1).value;

x(1) = pos(1).value(1,4);
x(1) = max(model.xmin,x(1));
x(1) = min(model.xmax,x(1));

y(1) = pos(1).value(2,4);
y(1) = max(model.ymin,y(1));
y(1) = min(model.ymax,y(1));

z(1) = pos(1).value(3,4);
z(1) = max(model.zmin,z(1));
z(1) = min(model.zmax,z(1));

for i=2:model.n
   
   T(i).value = T(i-1).value*TransfomationMatrix(r(i),phi(i),psi(i));
   pos(i).value = startPosition*T(i).value;
  
   x(i) = pos(i).value(1,4);
   x(i) = max(model.xmin,x(i));
   x(i) = min(model.xmax,x(i));
   
   y(i) = pos(i).value(2,4);
   y(i) = max(model.ymin,y(i));
   y(i) = min(model.ymax,y(i));
   
   z(i) = pos(i).value(3,4);
   z(i) = max(model.zmin,z(i));
   z(i) = min(model.zmax,z(i));

end

position.x = x;
position.y = y;
position.z = z;
end