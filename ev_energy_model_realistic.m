clc;
clear;
close all;

%% =================================================
%% PART 1: VEHICLE PARAMETERS
%% =================================================
m = 1800;                 % Vehicle mass (kg)
g = 9.81;                 % Gravity (m/s^2)
Crr = 0.01;               % Rolling resistance coefficient
rho = 1.225;              % Air density (kg/m^3)
Cd = 0.29;                % Drag coefficient
A = 2.2;                  % Frontal area (m^2)

delta = 0.08;             % Rotational Mass factor
m_eq = m*(1 + delta);

r_wheel = 0.3;            % Wheel radius (m)

% BATTERY PARAMETERS

Batt_capacity = 50;   % Battery capacity (kWh)

SOC_init = 0.9;       % Initial SOC (90%)

%% =================================================
%% PART 2: IMPORT UDDS DRIVE CYCLE
%% =================================================
udds_data = readtable("uddsdc.csv");

t = udds_data.UDDS;       % Time (s)
v_mph = udds_data.Var2;   % Speed (mph)

v = v_mph * 0.44704;      % Convert mph → m/s

dt = mean(diff(t));

a = [diff(v)/dt; 0];      % Acceleration

% Acceleration limits
a_max = 3;
a_min = -4;

a(a>a_max) = a_max;
a(a<a_min) = a_min;

%% =================================================
%% PART 3: LONGITUDINAL FORCES
%% =================================================

gear_ratio = 9;

omega_wheel = v / r_wheel;
omega = omega_wheel * gear_ratio;

F_rr   = m * g * Crr;
F_aero = 0.5 * rho * Cd * A .* v.^2;
F_acc  = m_eq .* a;

theta = deg2rad(0);
F_rg = m*g*sin(theta);

F_total = F_rr + F_aero + F_acc + F_rg;

%% =================================================
%% PART 4: WHEEL MECHANICAL POWER
%% =================================================

P_wheel = F_total .* v;

eta_tire = 0.95;
P_wheel = P_wheel / eta_tire;

%% =================================================
%% PART 5: MOTOR + REGEN LIMITS
%% =================================================

T_motor_max = 250;
T_regen_max = 120;
P_motor_max = 80e3;

v_min_regen = 1.5;

eta_drivetrain = 0.95;

P_mech = zeros(size(P_wheel));

omega_base = P_motor_max / T_motor_max;

for i = 1:length(P_wheel)

    if P_wheel(i) > 0

        if omega(i) <= omega_base
            T_avail = T_motor_max;
        else
            T_avail = P_motor_max / omega(i);
        end

        P_avail = T_avail * omega(i);

        P_mech(i) = min(P_wheel(i), P_avail);

    elseif P_wheel(i) < 0 && v(i) > v_min_regen

        P_regen_limit = T_regen_max * omega(i);

        P_mech(i) = -min(abs(P_wheel(i)), P_regen_limit);

    else

        P_mech(i) = 0;

    end

end

% drivetrain efficiency
P_mech = P_mech / eta_drivetrain;

%% =================================================
%% PART 6: DISTANCE & IDEAL ENERGY
%% =================================================

E_Wh = trapz(t, P_mech) / 3600;

dist_km = trapz(t, v) / 1000;

E_Wh_per_km = E_Wh / dist_km;

%% =================================================
%% PART 7: MOTOR + INVERTER EFFICIENCY
%% =================================================

omega_max = max(omega) + eps;

drive_eff = ones(size(P_mech));
regen_eff = ones(size(P_mech));

for i = 1:length(P_mech)

    if P_mech(i) > 0

        T_req = P_mech(i) / max(omega(i),eps);

        Tn = abs(T_req) / T_motor_max;

        on = omega(i) / omega_max;

        drive_eff(i) = 0.90 ...
            - 0.15*(1-on)^2 ...
            - 0.20*(Tn-0.6)^2;

        drive_eff(i) = min(max(drive_eff(i),0.75),0.92);

    elseif P_mech(i) < 0

        T_req = abs(P_mech(i)) / max(omega(i),eps);

        Tn = T_req / T_regen_max;

        on = omega(i) / omega_max;

        regen_eff(i) = 0.75 ...
            - 0.20*(1-on)^2 ...
            - 0.30*(Tn-0.5)^2;

        regen_eff(i) = min(max(regen_eff(i),0.55),0.80);

    end

end

%% =================================================
%% PART 8A: BATTERY POWER FLOW
%% =================================================

P_batt_disch_max = 90e3;
P_batt_char_max  = 40e3;

P_accessory = 900;

P_drive = zeros(size(P_mech));
P_regen = zeros(size(P_mech));

for i = 1:length(P_mech)

    if P_mech(i) > 0

        P_drive(i) = P_mech(i) / drive_eff(i);

        P_drive(i) = min(P_drive(i), P_batt_disch_max);

    elseif P_mech(i) < 0

        P_regen(i) = abs(P_mech(i)) * regen_eff(i);

        P_regen(i) = min(P_regen(i), P_batt_char_max);

    end

end

P_batt = P_drive - P_regen + P_accessory;

%% =================================================
%% PART 8B: BATTERY THERMAL PARAMETERS
%% =================================================

V_nom = 350;        % Nominal battery voltage (V)

R_internal = 0.05;  % Internal resistance (Ohm)

C_th = 45000;       % Thermal capacitance (J/K)

R_th = 1.5;         % Thermal resistance (K/W)

T_amb = 25;         % Ambient temperature (°C)


% OCV-SOC MODEL
SOC_curve = [0 0.1 0.2 0.4 0.6 0.8 1.0];

OCV_curve = [300 320 335 350 365 380 400];   % voltage vs SOC % simple linear OCV model


% BATTERY CURRENT
I_batt = P_batt ./ V_nom;



% HEAT GENERATION
Q_heat = (I_batt.^2) * R_internal;



% BATTERY TEMPERATURE INITIALIZATION
T_batt = zeros(size(t));

T_batt(1) = T_amb;


% BATTERY ELECTRICAL MODEL
V_oc = 360;   % Open circuit voltage (V)



%% =================================================
%% PART 9A: SOC MODEL
%% =================================================

V_batt = zeros(size(t));
V_oc_vec = zeros(size(t));

SOC = zeros(size(t));

SOC(1) = SOC_init;

% INITIAL BATTERY VOLTAGE
V_oc_vec(1) = interp1(SOC_curve, OCV_curve, SOC_init,'linear','extrap');

V_batt(1) = V_oc_vec(1) - I_batt(1) * R_internal;


for i = 2:length(t)

    % Energy consumed during timestep
    E_step_Wh = P_batt(i) * dt / 3600;

    % SOC update
    SOC(i) = SOC(i-1) - E_step_Wh/(Batt_capacity*1000);

    % Keep SOC physically valid
    SOC(i) = max(0,min(1,SOC(i)));

    % OCV from SOC lookup
    V_oc_vec(i) = interp1(SOC_curve, OCV_curve, SOC(i),'linear','extrap');

    % Heat generation
    Q = Q_heat(i);

    % Heat dissipation
    Q_cooling = (T_batt(i-1) - T_amb) / R_th;

    % Temperature change
    dT = (Q - Q_cooling) / C_th;

    % Temperature update
    T_batt(i) = T_batt(i-1) + dT * dt;

    % Battery terminal voltage
    V_batt(i) = V_oc_vec(i) - I_batt(i) * R_internal;

end


% ENERGY CONSUMPTION

Energy_used_Wh = sum(P_batt)*dt/3600;

Energy_used_kWh = Energy_used_Wh/1000;


%% =================================================
%% PART 9B: KALMAN FILTER SOC ESTIMATOR
%% =================================================

% SENSOR NOISE MODEL
noise = randn(size(V_batt))*0.5;

V_meas = V_batt + noise;



SOC_est = zeros(size(t));

SOC_est(1) = SOC_init;

P = 1e-4;   % estimation covariance

Q = 1e-6;   % process noise

R = 0.5;    % measurement noise

for k = 2:length(t)

    % SOC prediction (coulomb counting)
    SOC_pred = SOC_est(k-1) - (I_batt(k)*dt)/(Batt_capacity*3600);

    P = P + Q;

    % Predicted OCV from SOC
    V_oc_pred = interp1(SOC_curve, OCV_curve, SOC_pred,'linear','extrap');

    % Measurement model sensitivity
    H = 100;

    % Kalman Gain
    K = P*H/(H*P*H + R);

    % Update step
    SOC_est(k) = SOC_pred + K*(V_meas(k) - V_oc_pred);

    % Physical SOC limits
    SOC_est(k) = max(0,min(1,SOC_est(k)));

    % Covariance update
    P = (1-K*H)*P;

end

%% =================================================
%% PART 10: PLOTS
%% =================================================

figure('Name','EV UDDS Simulation');

subplot(4,1,1)
plot(t,v*3.6,'r','LineWidth',1.5)
xlabel('Time (s)')
ylabel('Speed (km/h)')
title('UDDS Drive Cycle')
grid on

subplot(4,1,2)
plot(t,P_drive/1000,'b'); hold on
plot(t,-P_regen/1000,'g')
xlabel('Time (s)')
ylabel('Power (kW)')
legend('Drive','Regen')
title('Battery Power Flow')
grid on

subplot(4,1,3)
plot(t,T_batt,'m','LineWidth',1.5)
xlabel('Time (s)')
ylabel('Temperature (°C)')
title('Battery Temperature')
grid on

subplot(4,1,4)
plot(t,SOC*100,'k','LineWidth',1.5)
xlabel('Time (s)')
ylabel('SOC (%)')
title('Battery State of Charge')
grid on

% SOC ESTIMATION COMPARISON

figure

plot(t,SOC*100,'k','LineWidth',2); hold on
plot(t,SOC_est*100,'r--','LineWidth',2)

legend('True SOC','Estimated SOC')

xlabel('Time (s)')
ylabel('SOC (%)')

title('Kalman Filter SOC Estimation')

grid on

%% =================================================
%% PART 11: FINAL RESULTS
%% =================================================

E_drive_Wh = trapz(t,P_drive)/3600;

E_regen_Wh = trapz(t,P_regen)/3600;

E_batt_net = E_drive_Wh - E_regen_Wh;

E_brake_mech = trapz(t,abs(min(P_mech,0)))/3600;

regen_percentage = (E_regen_Wh/E_brake_mech)*100;

P_drive_max = max(P_drive)/1000;

P_regen_max = max(P_regen)/1000;

P_wheel_max = max(P_mech)/1000;

SOC_initial = SOC(1)*100;

SOC_final = SOC(end)*100;

SOC_delta = SOC_initial - SOC_final;

avg_speed_kmh = mean(v)*3.6;

%% =================================================
%% PRINT RESULTS
%% =================================================

fprintf('\nVEHICLE & DRIVE CYCLE\n\n');

fprintf('Vehicle Mass\n: %.0f kg\n\n',m);

fprintf('Total Distance Travelled\n: %.3f km\n\n',dist_km);

fprintf('Average Speed\n: %.2f km/h\n\n',avg_speed_kmh);

fprintf('Drive Cycle Duration\n: %.0f s\n\n',t(end));


fprintf('ENERGY CONSUMPTION\n\n');

fprintf('Wheel Mechanical Energy\n: %.2f Wh\n\n',E_Wh);

fprintf('Battery Energy Drawn\n: %.2f Wh\n\n',E_drive_Wh);

fprintf('Battery Energy Regenerated\n: %.2f Wh\n\n',E_regen_Wh);

fprintf('Net Battery Energy Used\n: %.2f Wh\n\n',E_batt_net);

fprintf('Specific Energy Consumption\n: %.2f Wh/km\n\n',E_batt_net/dist_km);


fprintf('POWER PERFORMANCE\n\n');

fprintf('Maximum Wheel Power\n: %.2f kW\n\n',P_wheel_max);

fprintf('Maximum Battery Drive Power\n: %.2f kW\n\n',P_drive_max);

fprintf('Maximum Regen Power\n: %.2f kW\n\n',P_regen_max);


fprintf('REGENERATIVE BRAKING\n\n');

fprintf('Mechanical Braking Energy\n: %.2f Wh\n\n',E_brake_mech);

fprintf('Regen Energy Recovery\n: %.2f %%\n\n',regen_percentage);


fprintf('BATTERY STATE OF CHARGE\n\n');

fprintf('Initial SOC\n: %.2f %%\n\n',SOC_initial);

fprintf('Final SOC\n: %.2f %%\n\n',SOC_final);

fprintf('Net SOC Change\n: %.3f %%\n\n',SOC_delta);


%% =================================================
%% MOTOR TORQUE SPEED CHARACTERISTIC
%% =================================================

omega_range = linspace(0, max(omega)*1.2, 500);

T_motor = zeros(size(omega_range));

for i = 1:length(omega_range)

    if omega_range(i) <= omega_base
        T_motor(i) = T_motor_max;
    else
        T_motor(i) = P_motor_max / omega_range(i);
    end

end

figure

plot(omega_range, T_motor,'LineWidth',2);

xlabel('Motor Speed (rad/s)');
ylabel('Torque (Nm)');
title('EV Motor Torque-Speed Curve');

grid on;

%% =================================================
%% MOTOR EFFICIENCY MAP
%% =================================================

speed_norm = omega/omega_max;

torque = P_mech ./ max(omega,eps);

torque_norm = abs(torque)/T_motor_max;

figure

scatter(speed_norm, torque_norm, 15, drive_eff,'filled')

colorbar

xlabel('Normalized Speed');
ylabel('Normalized Torque');

title('Motor Efficiency Map');

grid on;

%% =================================================
%% ENERGY FLOW ANALYSIS
%% =================================================

E_accessory = trapz(t, ones(size(t))*P_accessory)/3600;

fprintf('\nENERGY FLOW ANALYSIS\n\n');

fprintf('Drive Energy From Battery : %.2f Wh\n',E_drive_Wh);

fprintf('Recovered Regen Energy    : %.2f Wh\n',E_regen_Wh);

fprintf('Accessory Consumption     : %.2f Wh\n',E_accessory);

fprintf('Net Battery Consumption   : %.2f Wh\n',E_batt_net);



%% =================================================
%% RANGE ESTIMATION
%% =================================================

usable_SOC = 0.8 - 0.2;   % 80% to 20%

usable_energy = Batt_capacity * usable_SOC * 1000; % Wh

range_est = usable_energy / (E_batt_net/dist_km);

fprintf('\nESTIMATED EV RANGE\n\n');

fprintf('Estimated Range (UDDS Based) : %.1f km\n',range_est);


%% =================================================
%% BATTERY ELECTRICAL ANALYSIS
%% =================================================

V_batt_avg = mean(V_batt);

V_batt_max = max(V_batt);

V_batt_min = min(V_batt);

I_batt_max = max(abs(I_batt));

fprintf('\nBATTERY ELECTRICAL PERFORMANCE\n\n');

fprintf('Average Battery Voltage : %.2f V\n',V_batt_avg);

fprintf('Maximum Battery Voltage : %.2f V\n',V_batt_max);

fprintf('Minimum Battery Voltage : %.2f V\n',V_batt_min);

fprintf('Maximum Battery Current : %.2f A\n',I_batt_max);



%% =================================================
%% SOC ESTIMATION PERFORMANCE USING KALMAN FILTER
%% =================================================

SOC_error = SOC - SOC_est;

RMSE = sqrt(mean(SOC_error.^2))*100;

max_error = max(abs(SOC_error))*100;

fprintf('\nSOC ESTIMATION PERFORMANCE\n\n');

fprintf('Kalman Filter RMSE : %.3f %%\n',RMSE);

fprintf('Maximum SOC Estimation Error : %.3f %%\n',max_error);


%% =================================================
%% BATTERY THERMAL ANALYSIS
%% =================================================

T_max = max(T_batt);

T_min = min(T_batt);

T_rise = T_max - T_amb;

fprintf('\nBATTERY THERMAL PERFORMANCE\n\n');

fprintf('Maximum Battery Temperature : %.2f °C\n',T_max);

fprintf('Minimum Battery Temperature : %.2f °C\n',T_min);

fprintf('Temperature Rise : %.2f °C\n',T_rise);


%% =================================================
%% BATTERY ELECTRICAL VISUALIZATION
%% =================================================

figure('Name','Battery Electrical Behaviour')

subplot(2,1,1)

plot(t,V_batt,'b','LineWidth',1.5)

xlabel('Time (s)')
ylabel('Voltage (V)')
title('Battery Terminal Voltage')

grid on


subplot(2,1,2)

plot(t,I_batt,'r','LineWidth',1.5)

xlabel('Time (s)')
ylabel('Current (A)')
title('Battery Current')

grid on

% Thermal Visualization Improvement

figure

plot(t,T_batt,'LineWidth',2)

yline(40,'r--','Thermal Limit')

xlabel('Time (s)')
ylabel('Battery Temperature (°C)')

title('Battery Thermal Safety Monitoring')

grid on

figure

plot(t,V_oc_vec,'k','LineWidth',2); hold on
plot(t,V_batt,'r','LineWidth',1.5)

legend('Open Circuit Voltage','Terminal Voltage')

xlabel('Time (s)')
ylabel('Voltage (V)')

title('Battery Voltage Behaviour')

grid on
