%
% Calculate path cost
%

function cost=MyCost(sol,model,varmin)

    J_inf = inf;
    n = model.n; % n is the number of path node, not including start point
    H = model.H; % H is the map
    
    % Input solution
    x=sol.x;
    y=sol.y;
    z=sol.z;
    
    % Start location
    xs=model.start(1);
    ys=model.start(2);
    zs=model.start(3);
    
    % Final location
    xf=model.end(1);
    yf=model.end(2);
    zf=model.end(3);
    
    x_all = [xs x xf];
    y_all = [ys y yf];
    z_all = [zs z zf];
    
    N = size(x_all,2); % Full path length
    
    % Altitude wrt sea level = z_relative + ground_level
    z_abs = zeros(1,N);
    for i = 1:N
        z_abs(i) = z_all(i) + H(round(y_all(i)),round(x_all(i)));
    end
    
    %============================================
    % J1 - Cost for path length 
    Traj = 0;
    % rmax = varmax.r;
    for i = 1:N-1
        diff = [x_all(i+1) - x_all(i);y_all(i+1) - y_all(i);z_abs(i+1) - z_abs(i)];
        if norm(diff) <= varmin.r
            Traj = 0;
            break;
        end
        Traj = Traj + norm(diff);
    end
    PP = norm([xf yf zf]-[xs ys zs]);
    J1 = abs(1 - PP/Traj);

    %==============================================
    % J2 - threats/obstacles Cost   

    % Threats/Obstacles
    threats = model.threats;
    threat_num = size(threats,1);
    
    drone_size = 1;
    danger_dist = 10*drone_size;
    
    J2 = 0;
    n2 = 0;
    for i = 1:threat_num
        threat = threats(i,:);
        threat_x = threat(1);
        threat_y = threat(2);
        threat_radius = threat(4);
        for j = 1:N-1
            % Distance between projected line segment and threat origin
            dist = DistP2S([threat_x threat_y],[x_all(j) y_all(j)],[x_all(j+1) y_all(j+1)]);
            if dist > (threat_radius + drone_size + danger_dist) % No collision
                threat_cost = 0;
                % do nothing
            elseif dist < (threat_radius + drone_size)  % Collision
                threat_cost = J_inf;
            else  % danger
                threat_cost = 1 - (dist-drone_size-threat_radius)/danger_dist;
            end
            n2 = n2+1;
            J2 = J2 + threat_cost;
        end
    end
    J2 = J2/n2;

    %==============================================
    % J3 - Altitude cost
    % Note: In this calculation, z, zmin & zmax are heights with respect to the ground
    zmax = model.zmax;
    zmin = model.zmin;
    J3 = 0;
    n3 = 0;
    for i=1:n        
        if z(i) < 0   % crash into ground
            J3_node = J_inf;
        else
            J3_node = abs(z(i) - (zmax + zmin)/2); 
        end
        J3_node = J3_node/((zmax-zmin)/2);
        n3 = n3+1;
        J3 = J3 + J3_node;
    end
    J3 = J3/n3;

    %==============================================
    % J4 - Smooth cost
    J4 = 0;
    n4 = 0;
    %find the heading angle at i position
    for i = 1:N-2   
        % P(ij)P(i,j+1)        
        for j = i:-1:1
             segment1 = [x_all(j+1); y_all(j+1); z_abs(j+1)] - [x_all(j); y_all(j); z_abs(j)];
             if nnz(segment1) ~= 0 % returns the number of nonzero elements in matrix
                 break; % --> if point(j+1) and point(j) is not coincide  
             end
        end
        
        % P(i,j+1)P(i,j+2)
        for j = i:N-2
            segment2 = [x_all(j+2); y_all(j+2); z_abs(j+2)] - [x_all(j+1); y_all(j+1); z_abs(j+1)];
             if nnz(segment2) ~= 0 
                 break;
             end
        end

        heading_angle = atan2(norm(cross(segment1,segment2)),dot(segment1,segment2));

        heading_angle = abs(heading_angle)/pi;
        n4 = n4+1;
        J4 = J4 + abs(heading_angle);
%         J4 = J4 + abs(climb_angle);
    end
    J4 = J4/n4;

    %============================================
    % Overall cost

    cost = [J1;J2;J3;J4];
end