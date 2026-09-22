%% analyse_capteurs.m
% Analyse de mesures capteurs (exportees depuis l'Arduino via le port
% serie, voir tests/log_capteurs_exemple.csv) : filtrage, calcul d'erreur
% et comparaison brut / filtre.
clear; clc; close all;

%% Chargement (ou generation si le fichier de log n'existe pas)
fichier_log = '../tests/log_capteurs_exemple.csv';
if isfile(fichier_log)
    data = readtable(fichier_log);
    t = data.temps_s;
    d_brut = data.distance_cm;
else
    % Generation d'un jeu de donnees synthetique realiste (HC-SR04)
    Ts = 0.06; t = 0:Ts:10;
    d_vraie = 60 + 30*sin(2*pi*0.1*t);
    bruit = 1.5*randn(size(t));
    aberrants = zeros(size(t));
    idx_aberrants = randi(length(t), 1, 8);
    aberrants(idx_aberrants) = (rand(1,8)-0.5)*80;
    d_brut = d_vraie + bruit + aberrants;
end

%% Filtrage - moyenne glissante + rejet de valeurs aberrantes (median filter)
fenetre = 5;
d_median = movmedian(d_brut, fenetre);
d_filtre = movmean(d_median, fenetre);

%% Metriques
d_ref = movmean(d_brut, 21); % reference "lente" pour comparaison
mae  = mean(abs(d_filtre - d_ref));
rmse = sqrt(mean((d_filtre - d_ref).^2));
mae_brut  = mean(abs(d_brut - d_ref));
rmse_brut = sqrt(mean((d_brut - d_ref).^2));

fprintf('--- Capteur ultrason HC-SR04 ---\n');
fprintf('MAE  signal brut   : %.2f cm\n', mae_brut);
fprintf('RMSE signal brut   : %.2f cm\n', rmse_brut);
fprintf('MAE  signal filtre : %.2f cm\n', mae);
fprintf('RMSE signal filtre : %.2f cm\n', rmse);
fprintf('Reduction du bruit : %.1f %%\n', 100*(1-rmse/rmse_brut));

%% Trace
figure('Name','Analyse capteur ultrason','Color','w');
plot(t, d_brut, 'Color',[0.7 0.7 0.7]); hold on;
plot(t, d_median, 'g-', 'LineWidth',1);
plot(t, d_filtre, 'b-', 'LineWidth',1.6);
grid on; legend('Mesure brute','Filtre median','Filtre median+moyenne','Location','best');
xlabel('Temps (s)'); ylabel('Distance (cm)');
title('Filtrage des mesures du capteur ultrasonique HC-SR04');
saveas(gcf, '../docs/figures/matlab_analyse_capteur_ultrason.png');

%% Histogramme des erreurs
figure('Name','Distribution des erreurs','Color','w');
histogram(d_brut-d_ref, 20, 'FaceColor',[0.8 0.3 0.3], 'FaceAlpha',0.6); hold on;
histogram(d_filtre-d_ref, 20, 'FaceColor',[0.2 0.4 0.8], 'FaceAlpha',0.6);
legend('Erreur signal brut','Erreur signal filtre');
xlabel('Erreur (cm)'); ylabel('Occurrences'); grid on;
title('Distribution des erreurs de mesure avant/apres filtrage');
saveas(gcf, '../docs/figures/matlab_histogramme_erreurs.png');
