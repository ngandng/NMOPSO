%
% Calculate path cost
%

function cost=MyCost(sol,model,varmax)
    
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
    J1 = 0;
    rmax = varmax.r;
    for i = 1:N-1
        diff = [x_all(i+1) - x_all(i);y_all(i+1) - y_all(i);z_abs(i+1) - z_abs(i)];
        if norm(diff)>rmax
            J1 = J_inf;
        else
            J1 = J1 + norm(diff);
        end
%         J1 = J1 + norm(diff);
    end

    %==============================================
    % J2 - threats/obstacles Cost   

    % Threats/Obstacles
    threats = model.threats;
    threat_num = size(threats,1);
    
    drone_size = 1;
    danger_dist = 10*drone_size;
    
    J2 = 0;
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
                threat_cost = (threat_radius + drone_size + danger_dist) - dist;
            end
            
            J2 = J2 + threat_cost;
        end
    end

    %==============================================
    % J3 - Altitude cost
    % Note: In this calculation, z, zmin & zmax are heights with respect to the ground
    zmax = model.zmax;
    zmin = model.zmin;
    J3 = 0;
    for i=1:n        
        if z(i) < 0   % crash into ground
            J3_node = J_inf;
        else
            J3_node = abs(z(i) - (zmax + zmin)/2); 
        end
        
        J3 = J3 + J3_node;
    end
    
    %==============================================
    % J4 - Smooth cost
    J4 = 0;
    turning_max = varmax.phi;
    climb_max = varmax.psi;
    
    for i = 1:N-2 %find the climbing and turning angle at i+1 position
        
        for j = i:-1:1
             segment1 = [x_all(j+1); y_all(j+1); z_abs(j+1)] - [x_all(j); y_all(j); z_abs(j)];
             if nnz(segment1) ~= 0 % return the number of nonzero in matrix
                 break; % --> if point(j+1) and point(j) is coincide  
             end
        end
        % Projection of line segments to Oxy(j+1)
        for j = i:N-2
            segment2 = [x_all(j+2); y_all(j+2); z_abs(j+1)] - [x_all(j+1); y_all(j+1); z_abs(j+1)];
             if nnz(segment2) ~= 0 
                 break;
             end
        end
        
        climb_angle = atan2(z_abs(i+1) - z_abs(i),norm(segment2));
       
        turning_angle = atan2(norm(cross(segment1,segment2)),dot(segment1,segment2));
       
%         if abs(turning_angle) > turning_max
%             J4 = J4 + abs(turning_angle);
%         end
%         if abs(climb_angle) > climb_max
%             J4 = J4 + abs(climb_angle);
%         end
        J4 = J4 + abs(turning_angle);
        J4 = J4 + abs(climb_angle);
       
    end

    %============================================
    % Weight coeffcients
    b1 = 5;
    b2 = 1;
    b3 = 10;
    b4 = 1;
    % Overall cost
    cost = [J1;J2;J3;J4];
end