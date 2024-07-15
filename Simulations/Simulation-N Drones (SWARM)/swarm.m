% Parameters
N = 5; 
master_index = 3;
positions = [0, 0; 10, 20; 20, 40; 80, 60; 100, 80]; 
m = 1.8;  


tspan = [0 200]; 

% Initial conditions
initial_conditions = reshape([positions, zeros(N, 2)]', [], 1);


function dstate_dt = odesystem(t, state, N, m, master_index)
    d_coeff = 0.1;
    Kx = 1;
    Ky = 1;
    K1 = 2;
    K2 = 2;
    
    dstate_dt = zeros(4*N, 1);
    for i = 1:N
        if i == master_index
            continue; % Skip updating the master drone's position
        end
        
        x_i = state((i-1)*4 + 1);
        y_i = state((i-1)*4 + 2);
        V_x_i = state((i-1)*4 + 3);
        V_y_i = state((i-1)*4 + 4);
        
        F_x = 0;
        F_y = 0;
        
        for j = 1:N
            if i ~= j
                x_j = state((j-1)*4 + 1);
                y_j = state((j-1)*4 + 2);
                
                r = sqrt((x_i - x_j)^2 + (y_i - y_j)^2);
                theta = atan2(y_j - y_i, x_j - x_i);
                
                F = -7.2*((10/r)^4 - (10/r)^2);
                
                F_x = F_x + Kx * F * cos(theta);
                F_y = F_y + Ky * F * sin(theta);
            end
        end
        
        dx_dt = V_x_i;
        dy_dt = V_y_i;
        dV_x_dt = K1 * (F_x - V_x_i);
        dV_y_dt = K2 * (F_y - V_y_i);
        
        dstate_dt((i-1)*4 + 1) = dx_dt;
        dstate_dt((i-1)*4 + 2) = dy_dt;
        dstate_dt((i-1)*4 + 3) = dV_x_dt;
        dstate_dt((i-1)*4 + 4) = dV_y_dt;
    end
end

% Function handle to pass additional parameters
odefun = @(t, state) odesystem(t, state, N, m, master_index);

% Solve the system of ODEs
[t, result] = ode45(odefun, tspan, initial_conditions);

% Extract the results
x_values = result(:, 1:4:end);
y_values = result(:, 2:4:end);

% Plot the motion of the drones
figure;
hold on;
colors = lines(N);
h = gobjects(N, 1);
paths = gobjects(N, 1);
for i = 1:N
    h(i) = plot(positions(i, 1), positions(i, 2), 'o', 'MarkerSize', 5, 'Color', colors(i,:), 'DisplayName', sprintf('Drone %d', i));
    paths(i) = plot(x_values(1, i), y_values(1, i), '-', 'Color', colors(i,:), 'DisplayName', sprintf('Path %d', i));
end
xlabel('X');
ylabel('Y');
title('Motion of the Drones');
legend show;
grid on;
axis equal;

% Animate the motion
for k = 1:length(t)
    for i = 1:N
        if i == master_index
            continue; % Skip updating the master drone's position
        end
        set(h(i), 'XData', x_values(k, i), 'YData', y_values(k, i));
        set(paths(i), 'XData', x_values(1:k, i), 'YData', y_values(1:k, i));
    end
    pause(0.05);
end

hold off;
