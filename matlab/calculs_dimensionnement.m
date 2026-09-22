%% calculs_dimensionnement.m
% Reproduit tous les calculs justificatifs du rapport d'ingenierie
% (docs/01_rapport_dimensionnement.md) et affiche un tableau de synthese.
%
% Auteur : Projet Voiture & Drone Autonomes
clear; clc;

fprintf('=========================================================\n');
fprintf(' DIMENSIONNEMENT - VOITURE AUTONOME\n');
fprintf('=========================================================\n');

% --- Parametres ---
m_v   = 1.20;      % kg, masse totale
g     = 9.81;       % m/s^2
d     = 0.065;       % m, diametre roue
r     = d/2;          % m, rayon roue
Crr   = 0.02;         % coefficient de resistance au roulement
v_max = 0.50;        % m/s
a_v   = 0.30;         % m/s^2

F_roll = m_v*g*Crr;
F_acc  = m_v*a_v;
F_tot  = F_roll + F_acc;
F_par_roue = F_tot/2;
T_roue = F_par_roue*r;         % N.m
omega  = v_max/r;              % rad/s
rpm    = omega*60/(2*pi);

fprintf('Force resistance roulement   F_roll = %.3f N\n', F_roll);
fprintf('Force acceleration           F_acc  = %.3f N\n', F_acc);
fprintf('Couple requis par roue       T_roue = %.2f mN.m\n', T_roue*1e3);
fprintf('Vitesse angulaire requise    omega  = %.2f rad/s (%.1f tr/min)\n', omega, rpm);

% --- Autonomie voiture ---
I_moteurs = 4*0.180;   % A
I_arduino = 0.050;
I_capteurs= 0.080;
I_marge   = 0.150;
I_tot_v   = I_moteurs + I_arduino + I_capteurs + I_marge;
C_batt_v  = 2.0;       % Ah
eta_v     = 0.70;
t_theo_v  = C_batt_v/I_tot_v;
t_reel_v  = t_theo_v*eta_v;

fprintf('Courant total consomme       I_tot  = %.3f A\n', I_tot_v);
fprintf('Autonomie theorique                 = %.2f h\n', t_theo_v);
fprintf('Autonomie reelle (eta=%.0f%%)         = %.2f h (%.0f min)\n\n', eta_v*100, t_reel_v, t_reel_v*60);

fprintf('=========================================================\n');
fprintf(' DIMENSIONNEMENT - DRONE AUTONOME (quadricoptere)\n');
fprintf('=========================================================\n');

m_d   = 0.90;          % kg, MTOW
TW    = 2.0;           % rapport poussee/poids vise
n_mot = 4;

P_d       = m_d*g;
T_tot     = TW*P_d;
T_par_mot = T_tot/n_mot;

fprintf('Poids total                  P      = %.2f N\n', P_d);
fprintf('Poussee totale requise (T/W=%.1f) T_tot = %.2f N (%.0f g)\n', TW, T_tot, T_tot/g*1000);
fprintf('Poussee par moteur            T_mot = %.2f N (%.0f g)\n', T_par_mot, T_par_mot/g*1000);

% --- Autonomie drone ---
I_hover   = 4*4.0;     % A, estimation hover
C_batt_d  = 1.5;        % Ah
DoD       = 0.8;        % profondeur de decharge admissible
t_hover_h = (C_batt_d*DoD)/I_hover;
t_hover_min = t_hover_h*60;

fprintf('Courant estime au hover       I_hov = %.1f A\n', I_hover);
fprintf('Autonomie vol stationnaire           = %.2f min\n\n', t_hover_min);

% --- Tableau de synthese ---
fprintf('=========================================================\n');
fprintf(' TABLEAU DE SYNTHESE DES MARGES\n');
fprintf('=========================================================\n');
T = table(...
    ["Voiture - couple moteur";"Voiture - autonomie";"Drone - poussee T/W";"Drone - autonomie hover"], ...
    [T_roue*1e3; t_reel_v*60; T_tot/P_d; t_hover_min], ...
    ["mN.m";"min";"-";"min"], ...
    'VariableNames', {'Grandeur','Valeur','Unite'});
disp(T);

%% Figure recapitulative
figure('Name','Synthese dimensionnement','Color','w');
subplot(1,2,1);
bar([F_roll F_acc F_tot]);
set(gca,'XTickLabel',{'F_{roll}','F_{acc}','F_{tot}'});
ylabel('Force (N)'); title('Voiture : bilan des forces'); grid on;

subplot(1,2,2);
bar([P_d T_tot T_par_mot*4]);
set(gca,'XTickLabel',{'Poids','Poussee tot.','4xPoussee mot.'});
ylabel('Force (N)'); title('Drone : bilan poussee/poids'); grid on;

saveas(gcf, '../docs/figures/matlab_dimensionnement.png');
