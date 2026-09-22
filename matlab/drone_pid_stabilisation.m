%% drone_pid_stabilisation.m
% Simulation de la boucle de stabilisation en attitude (roulis) d'un
% quadricoptere : architecture cascade (boucle angle -> boucle taux)
% avec rejet de perturbation (rafale de vent simulee).
clear; clc; close all;

%% Modele dynamique simplifie (axe de roulis)
% theta_ddot = (1/Ixx) * tau_roll
Ixx = 0.0045;         % kg.m^2, inertie autour de l'axe de roulis (estimee)
G_taux  = tf(1/Ixx, [1 0]);      % taux angulaire p(s)/tau(s) = 1/(Ixx.s)
G_angle = tf(1, [1 0]);           % angle theta(s)/p(s) = 1/s

%% Boucle interne : regulateur de taux angulaire (PD rapide)
Kp_taux = 0.70; Kd_taux = 0.015;
C_taux = pid(Kp_taux, 0, Kd_taux);
BF_taux = feedback(C_taux*G_taux, 1);

%% Boucle externe : regulateur d'angle (PI lent) autour de la boucle taux fermee
Kp_angle = 4.5; Ki_angle = 0.2;
C_angle = pid(Kp_angle, Ki_angle);
BF_angle = feedback(C_angle*BF_taux*G_angle, 1);

%% Reponse a un echelon de consigne (5 deg)
figure('Name','Stabilisation roulis - reponse indicielle','Color','w');
step(5*BF_angle, 1.0);
grid on;
title(sprintf('Reponse en roulis (Kp_{angle}=%.1f, Ki_{angle}=%.1f, Kp_{taux}=%.2f, Kd_{taux}=%.3f)', ...
      Kp_angle, Ki_angle, Kp_taux, Kd_taux));
ylabel('Angle de roulis (deg)'); xlabel('Temps (s)');
saveas(gcf, '../docs/figures/matlab_drone_stabilisation.png');

info = stepinfo(5*BF_angle);
fprintf('Depassement        : %.2f %%\n', info.Overshoot);
fprintf('Temps stabilisation: %.3f s\n', info.SettlingTime);

%% Simulation temporelle avec perturbation (rafale de vent a t=1s)
Ts = 0.004;    % 250 Hz, frequence boucle reelle
t  = 0:Ts:3;
N  = length(t);

consigne = zeros(1,N);
consigne(t>=0.2) = 5;          % consigne 5 deg a partir de t=0.2s

perturbation = zeros(1,N);
perturbation(t>=1.5 & t<1.7) = 15; % rafale de vent = couple perturbateur (deg equiv.)

theta = zeros(1,N); p = zeros(1,N);
int_angle = 0; int_taux = 0; prev_e_taux = 0;

P_CONS_MAX = 300;   % deg/s, limite d'autorite de la boucle angle (anti-emballement)
TAU_MAX    = 0.05;  % N.m, limite d'autorite du moteur (saturation actionneur)

for k = 2:N
    e_angle = consigne(k) - theta(k-1);
    int_angle = int_angle + e_angle*Ts;
    p_cons = Kp_angle*e_angle + Ki_angle*int_angle;   % consigne de taux
    p_cons = max(min(p_cons, P_CONS_MAX), -P_CONS_MAX); % saturation + anti-windup

    e_taux = p_cons - p(k-1);
    d_taux = (e_taux - prev_e_taux)/Ts;
    tau_cmd = Kp_taux*e_taux + Kd_taux*d_taux;
    tau_cmd = max(min(tau_cmd, TAU_MAX), -TAU_MAX);     % saturation actionneur
    prev_e_taux = e_taux;

    p(k) = p(k-1) + (tau_cmd/Ixx)*Ts + perturbation(k)*0.02;
    theta(k) = theta(k-1) + p(k)*Ts;
end

figure('Name','Stabilisation roulis avec rafale de vent','Color','w');
plot(t, consigne, 'k--', t, theta, 'b-', 'LineWidth',1.3); hold on;
xline(1.5,'r:','Rafale de vent');
grid on; legend('Consigne','Angle mesure (simule)','Location','best');
ylabel('Angle de roulis (deg)'); xlabel('Temps (s)');
title('Rejet de perturbation - boucle cascade angle/taux');

saveas(gcf, '../docs/figures/matlab_drone_rejet_perturbation.png');

erreur_max_transitoire = max(abs(theta(t>=1.5 & t<=1.9) - 5));
fprintf('Erreur max pendant la rafale : %.2f deg\n', erreur_max_transitoire);
fprintf('Temps de retour a +/-1 deg apres rafale : voir figure.\n');
