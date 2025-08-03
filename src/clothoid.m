% MATLAB script to generate a track with 4 clothoids and 4 straight segments + export points to CSV

% General parameters
R         = 50;                % Final circle radius [m]
k_end     = 1/R;               % Final curvature = 1/R
L         = 80;                % Length of each clothoid [m]
n         = 100;               % Number of integration points per clothoid
s         = linspace(0, L, n); % Arc-length parameter
c         = k_end / L;         % Curvature coefficient for s

% Compute the accumulated angle for numerical integration
theta = c * s.^2;               % θ(s) = ∫₀ˢ (c · u) du = c · s²

% Compute cos(theta) and sin(theta)
cos_t = cos(theta);
sin_t = sin(theta);

% Generate 4 clothoids at specified offsets

% Clothoid 1 (starting at origin, mirrored in x)
x1 = -cumtrapz(s, cos_t);
y1 =  cumtrapz(s, sin_t);

% Clothoid 2 (shifted +200 in x)
x2 = 200 + cumtrapz(s, cos_t);
y2 =       cumtrapz(s, sin_t);

% Clothoid 3 (shifted +200 in x, mirrored in y)
x3 = 200 + cumtrapz(s, cos_t);
y3 = 100 - cumtrapz(s, sin_t);

% Clothoid 4 (shifted +0 in x, +100 in y, mirrored in both)
x4 =       -cumtrapz(s, cos_t);
y4 = 100 - cumtrapz(s, sin_t);

% Create straight segments to connect the clothoids
n_st    = 200;  % Number of points for long straight segments
n_short = 50;   % Number of points for shorter straights

% Straight 1→2
x12 = linspace(x1(1), x2(1), n_st);
y12 = linspace(y1(1), y2(1), n_st);

% Straight 2→3
x23 = linspace(x2(end), x3(end), n_short);
y23 = linspace(y2(end), y3(end), n_short);

% Straight 3→4
x34 = linspace(x3(1), x4(1), n_st);
y34 = linspace(y3(1), y4(1), n_st);

% Straight 4→1 (to close the loop)
x41 = linspace(x4(end), x1(end), n_short);
y41 = linspace(y4(end), y1(end), n_short);

% ---------------------------
% Plot the complete track
% ---------------------------
figure;
hold on;
plot(x1,  y1,  'r',  'LineWidth', 2);
plot(x2,  y2,  'g',  'LineWidth', 2);
plot(x3,  y3,  'b',  'LineWidth', 2);
plot(x4,  y4,  'm',  'LineWidth', 2);

plot(x12, y12, 'k--', 'LineWidth', 1.5);
plot(x23, y23, 'k--', 'LineWidth', 1.5);
plot(x34, y34, 'k--', 'LineWidth', 1.5);
plot(x41, y41, 'k--', 'LineWidth', 1.5);

hold off;
axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Track with 4 Clothoids and 4 Straight Segments');
legend({'clothoid 1','clothoid 2','clothoid 3','clothoid 4', ...
        'straight 1–2','straight 2–3','straight 3–4','straight 4–1'}, ...
       'Location','best');

% ---------------------------
% Export all generated points to CSV
% ---------------------------
X = [ x1(:)
      x12(:)
      x2(:)
      x23(:)
      x3(:)
      x34(:)
      x4(:)
      x41(:) ];

Y = [ y1(:)
      y12(:)
      y2(:)
      y23(:)
      y3(:)
      y34(:)
      y41(:) 
      y4(:)];

points = [X, Y];

% Write as a table with headers
T = array2table(points, 'VariableNames', {'x','y'});
writetable(T, 'track_points.csv');

disp('Saved track_points.csv successfully!');
