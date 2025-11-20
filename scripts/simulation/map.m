% ==== ellipse_on_map.m ====
% Draw an ellipse with foci at WEiTI and PKiN on a geographic map

% Coordinates of the foci
latF1 = 52.2193;  lonF1 = 21.0110;   % WEiTI
latF2 = 52.2319;  lonF2 = 21.0067;   % PKiN
latF1 = deg2rad(latF1);
lonF1 = deg2rad(lonF1);
latF2 = deg2rad(latF2);
lonF2 = deg2rad(lonF2);
% Convert lat/lon to Cartesian (approximate, small region)
% 1 degree latitude ≈ 111 km, 1 degree longitude ≈ 111*cos(latitude) km
R = 6371;   % Radius of the earth [km]

x1 = R * cos(latF1) * cos(lonF1);
x2 = R * cos(latF2) * cos(lonF2);

y1 = R * cos(latF1) * sin(lonF1);
y2 = R * cos(latF2) * sin(lonF2);

z1 = R * sin(latF1);
z2 = R * sin(latF2);

Rb = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);


% Choose ellipse major axis length (km)
a = 0.5;  % half of major axis, must be > c
b = sqrt(a^2 - Rb^2);

% Ellipse parametric equation centered between foci
theta = linspace(0, 2*pi, 400);
x = a * cos(theta);
y = b * sin(theta);

% Rotate ellipse to align with foci
angle = atan2(y1, x1);
xRot = x * cos(angle) - y * sin(angle);
yRot = x * sin(angle) + y * cos(angle);

% Shift center between foci (in lat/lon)
centerLat = mean([latF1, latF2]);
centerLon = mean([lonF1, lonF2]);

% Convert back to lat/lon
latEllipse = centerLat + yRot / 111;
lonEllipse = centerLon + xRot / (111 * cosd(lat0));

% Plot on map
figure
gx = geoaxes;
geobasemap streets
hold(gx, 'on')

% Plot ellipse
geoplot(gx, latEllipse, lonEllipse, 'r-', 'LineWidth', 2)
% Optional fill:
% geopolygon(gx, latEllipse, lonEllipse, 'FaceColor', [1 0 0], 'FaceAlpha', 0.2)

% Plot foci
geoplot(gx, latF1, lonF1, 'bo', 'MarkerFaceColor', 'b')
text(latF1, lonF1, ' WEiTI', 'FontSize', 10, 'Color', 'b')

geoplot(gx, latF2, lonF2, 'go', 'MarkerFaceColor', 'g')
text(latF2, lonF2, ' PKiN', 'FontSize', 10, 'Color', 'g')

title('Ellipse with Foci at WEiTI and PKiN (Warsaw)')
