clear; clc; close all;

%% 1. Konfiguracja
L = 1500; % Długość bazy [m]
R_sums = [1600, 1800, 2200, 2800, 3600]; % Izolinie

Tx_pos = [-L/2, 0];
Rx_pos = [L/2, 0];
theta = linspace(0, 2*pi, 1000);

%% 2. Ustawienia Wykresu (Dark Mode)
figure('Name', 'Elipsy Bistatyczne (Dark)', ...
       'Color', 'k', ...                 % Czarne tło okna
       'Position', [100, 100, 900, 700]);

ax = gca;
hold on; axis equal; grid on;

% Stylizacja osi na biało
set(ax, 'Color', 'k');          % Czarne tło wykresu
set(ax, 'XColor', 'w');         % Białe osie X
set(ax, 'YColor', 'w');         % Białe osie Y
set(ax, 'GridColor', 'w');      % Biała siatka
set(ax, 'GridAlpha', 0.25);     % Przezroczystość siatki (żeby nie raziła)
set(ax, 'LineWidth', 1.2);

% Paleta kolorów (jasna, dobrze widoczna na czarnym)
colors = spring(length(R_sums) + 2); 

%% 3. Rysowanie Elips
for i = 1:length(R_sums)
    Rb = R_sums(i);
    a = Rb / 2;
    c = L / 2;
    
    if a <= c, continue; end
    b = sqrt(a^2 - c^2);
    
    x = a * cos(theta);
    y = b * sin(theta);
    
    % Rysowanie linii
    plot(x, y, 'LineWidth', 1.5, 'Color', colors(i,:));
    
    % --- INTELIGENTNE POZYCJONOWANIE NAPISÓW ---
    % Zmieniamy kąt dla każdej kolejnej elipsy, aby napisy się nie nakładały.
    % Startujemy od 10 stopni i przesuwamy o 12 stopni dla każdej kolejnej.
    angle_deg = 10 + (i * 12); 
    
    % Konwersja kąta na indeks w wektorze theta (0..360 -> 1..1000)
    idx = round((angle_deg / 360) * length(theta));
    
    % Pobranie współrzędnych
    x_lbl = x(idx);
    y_lbl = y(idx);
    
    label_str = sprintf('R=%dm', Rb);
    
    % Dodanie tekstu
    text(x_lbl, y_lbl, label_str, ...
        'Color', colors(i,:), ...       % Kolor tekstu taki sam jak elipsy
        'BackgroundColor', 'k', ...     % Czarne tło pod tekstem (zasłania linię i siatkę)
        'EdgeColor', 'none', ...        % Brak ramki (wygląda nowocześniej na czarnym)
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
end

%% 4. Rysowanie Tx, Rx i Bazy
% Linia bazy (biała, przerywana)
plot([Tx_pos(1), Rx_pos(1)], [0, 0], 'w--', 'LineWidth', 1, 'DisplayName', 'Baza');

% Tx (Czerwony trójkąt z białą obwódką)
plot(Tx_pos(1), Tx_pos(2), '^', 'MarkerSize', 11, ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(Tx_pos(1), -250, 'Tx', 'Color', 'r', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);

% Rx (Zielone koło z białą obwódką)
plot(Rx_pos(1), Rx_pos(2), 'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(Rx_pos(1), -250, 'Rx', 'Color', 'g', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);

%% 5. Opisy
xlabel('Odległość X [m]', 'Color', 'w', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Odległość Y [m]', 'Color', 'w', 'FontSize', 16, 'FontWeight', 'bold');
title(sprintf('Geometria Bistatyczna (Baza L = %d m)', L), ...
    'Color', 'w', 'FontSize', 18, 'FontWeight', 'bold');

% Marginesy widoku
max_R = max(R_sums);
xlim([-max_R/1.8, max_R/1.8]);
ylim([-max_R/2.2, max_R/2.2]);

hold off;