% Parameters
x1 = 0; % Leader point x-coordinate
y1 = 0; % Leader point y-coordinate
x2 = 11; % Initial follower point x-coordinate
y2 = 5; % Initial follower point y-coordinate
m = 1.8;  % Mass of the follower

% Time span for the simulation
tspan = [0 200]; % Adjust as needed

% Initial conditions: [x2, y2, V_x, V_y]
initial_conditions = [x2, y2, 0, 0];

% Define the system of ODEs
function dstate_dt = odesystem(t, state, x1, y1, m)
    x = state(1);
    y = state(2);
    V_x = state(3);
    V_y = state(4);
    d_coeff = 0.1;


    r = sqrt((x - x1)^2 + (y - y1)^2);
    theta = atan2(y - y1, x - x1);
    % F = -(1/r^2) + 2*r;

    F = 7.2*((10/r)^12 - (10/r)^6);

    
    dx_dt = V_x;
    dy_dt = V_y;
    dV_x_dt =  ((F / m) *cos(theta)- d_coeff*V_x);
    dV_y_dt =  ((F / m)*sin(theta)- d_coeff*V_y);
    
    dstate_dt = [dx_dt; dy_dt; dV_x_dt; dV_y_dt];
end

% Function handle to pass additional parameters
odefun = @(t, state) odesystem(t, state, x1, y1, m);

% Solve the system of ODEs
[t, result] = ode45(odefun, tspan, initial_conditions);

% Extract the results
x2_values = result(:, 1);
y2_values = result(:, 2);

% Plot the motion of the follower point
figure;
h = plot(x1, y1, 'ro', 'MarkerSize', 10, 'DisplayName', 'Leader Point');
hold on;
follower = plot(x2, y2, 'bo', 'MarkerSize', 5, 'DisplayName', 'Follower Point');
path = plot(x2_values(1), y2_values(1), 'b-', 'DisplayName', 'Follower Path');
xlabel('X');
ylabel('Y');
title('Motion of the Follower Point');
legend show;
grid on;
axis equal;
% xlim([min([x1, x2_values'])-1, max([x1, x2_values'])+1]);
% ylim([min([y1, y2_values'])-1, max([y1, y2_values'])+1]);

% Animate the motion
for k = 1:length(t)
    % Update the follower's position
    set(follower, 'XData', x2_values(k), 'YData', y2_values(k));
    
    % Update the path
    set(path, 'XData', x2_values(1:k), 'YData', y2_values(1:k));
    
    % Pause for a short duration to create animation effect
    pause(0.05);
end

hold off;
