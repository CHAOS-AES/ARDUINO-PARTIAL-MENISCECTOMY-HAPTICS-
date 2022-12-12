% Kalman filter force estimation simulation
% By Øystein Bjelland, CPS-Lab, IIR, NTNU
% Start date: 17.03.2022

close all;
clc;
clear all;

%% Import empirical data

filename1 = '11_meniscus_fast_1826 - Copy.txt';

i_start = 5900; % From data series
i_stop = 5975;

A = importdata(filename1);

inputAngle_raw = A(:,4);  %Input angle [deg]
outputFingerForce_raw = A(:,1);   %Output finger force [raw fsr reading]. Set to ":2" for [g]

time_raw = A(:,5);  %Raw time from millis() in Arduino [milliseconds]
time = zeros(length(time_raw),1);
Ts_vect = [];

Ts_ref = 5*(time_raw(3) - time_raw(2));

for i = 2:length(time_raw)
   Ts = time_raw(i) - time_raw(i-1); 
   % Preventing the time gap between the sampling series to artificially increase sampling time.
   if Ts <  Ts_ref
        Ts_vect = [Ts_vect, Ts];
   end  
   time(i) = (time_raw(i) - time_raw(1))/1000; %Starting the time vector from zero and converting from ms to s.
end

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

disp('Our sampling frequency is, Fs [Hz]')
Fs = 1/Ts;
disp(Fs)

inputAngle_filtered = lowpass(inputAngle_raw, 5, Fs); %Remember to try without this later
%inputAngle_filtered = inputAngle_raw;

theta_random_cut = inputAngle_filtered(i_start:i_stop);
Force_random_cut = outputFingerForce_raw(i_start:i_stop);

% Converting raw force data to newtons
outputFingerForce_newtons = zeros(1, length(outputFingerForce_raw));

for i = 1:length(outputFingerForce_raw)    
    % Conversion from calibration data in excel sheet, plus converting from
    % gram to newton
    outputFingerForce_newtons(i) = (3*10^(-7)*(outputFingerForce_raw(i))^3 - 0.0002*(outputFingerForce_raw(i))^2 + 0.7815*outputFingerForce_raw(i))*(10^(-3)*9.81);   
end

Force_random_cut_newtons = outputFingerForce_newtons(i_start:i_stop);


%% Stiffness check (ignore)

%theta_1 = inputAngle_filtered(1:150);
%Force_1 = outputFingerForce_newtons(1:150);

theta_2 = inputAngle_filtered(90:170);
Force_2 = outputFingerForce_newtons(90:170);


theta_3 = inputAngle_filtered(870:910);
Force_3 = outputFingerForce_newtons(870:910);

theta_4 = inputAngle_filtered(1440:1480);
Force_4 = outputFingerForce_newtons(1440:1480);
plot(theta_4, Force_4);

theta_5 = inputAngle_filtered(2300:2400);
Force_5 = outputFingerForce_newtons(2300:2400);

theta_6 = inputAngle_filtered(3000:3050);
Force_6 = outputFingerForce_newtons(3000:3050);

theta_7 = inputAngle_filtered(3700:3800);
Force_7 = outputFingerForce_newtons(3700:3800);

theta_8 = inputAngle_filtered(4300:4370);
Force_8 = outputFingerForce_newtons(4300:4370);

theta_9 = inputAngle_filtered(5100:5150);
Force_9 = outputFingerForce_newtons(5100:5150);

theta_10 = inputAngle_filtered(5900:5970);
Force_10 = outputFingerForce_newtons(5900:5970);

theta_11 = inputAngle_filtered(6800:6840);
Force_11 = outputFingerForce_newtons(6800:6840);

force_signal = [Force_2, Force_3, Force_4, Force_5, Force_6, Force_7, Force_8, Force_9, Force_10, Force_11]; %[N]
theta_signal = [theta_2; theta_3; theta_4; theta_5; theta_6; theta_7; theta_8; theta_9; theta_10; theta_11]; %[deg]
theta_signal = theta_signal';


stiffness = zeros(1, length(force_signal));
for i = 2:length(force_signal)
    stiffness(i) = (force_signal(i) - force_signal(i-1))/(theta_signal(i) - theta_signal(i-1));
end

figure(1)
plot(stiffness);
title('Stiffness over 10 cuts');
ylabel('Stiffness [N/\theta^{\circ}]');

%% Plot

figure(2)
plot(inputAngle_filtered, outputFingerForce_newtons,'k'); 
title('Partial Meniscectomy Punch Raw Data');
xlabel('Handle Position [\theta^{\circ}]');
ylabel('F_{FSR} [N]');

figure(3)
plot(theta_random_cut, Force_random_cut_newtons);
title('Selected Meniscectomy Punch Force Curve');
xlabel('Handle Position [\theta^{\circ}]');
ylabel('F_{FSR} [N]');

figure(4)
plot(theta_random_cut, Force_random_cut_newtons);
title('Selected Meniscectomy Punch Force Curve');
xlabel('Handle Position [\theta^{\circ}]');
ylabel('Force [N]');
hold on;


%% Simulate haptic force signal from Finite Element Simulation

E = (4.344 - 0) / (6.335 - 3.813); % Tissue stiffness [N/deg]
Force_FEM = zeros(1, length(Force_random_cut_newtons));
theta_stop = 10.81;
%theta_FEM = 0:theta_stop/(length(Force_random_cut_newtons)-1):theta_stop; 
theta_FEM = theta_random_cut;

for i = 1:length(Force_random_cut_newtons) 

    Force_FEM(i) = E*theta_FEM(i)-E*3.813;
    if (theta_FEM(i) < 3.813)
       Force_FEM(i) = 0;
    elseif (Force_FEM(i) > 5)
        Force_FEM(i) = 5;
    end
        
end

plot(theta_FEM, Force_FEM);
xlim([0, 12]);
ylim([0,8]);
hold on;


%% Kalman Filtering

N = length(Force_random_cut_newtons);
E = 1.7224; %Stiffness [N/deg]
E_array = ones(1,N)*E;

A = [1 1; 0 1];
B = [1; 0];
H = [1 0];
u = 0;

dTheta = 0.1441;

Ad = A; Ad(1,2) = dTheta;
Bd = B;

%Q = [0.1 0; 0 0.4];
%Q = [3.1248 4.70; 4.70 9.4481];
%Q = [0.001 0; 0 0.001];
Q = [7.0275 0.2772; 0.2772 6.1024];
%R_empirical = 3.1248;
%R_FEM = 5.2090;

R_empirical = 7.742;
%R_empirical = 0.9507; % From calibration data measurements
R_FEM = 0.001;

Pplus = zeros(2);

xhat = [0; 0];
xhatArray = [xhat];


force_measurement = zeros(2,N);

for i = 2:N
    
    xhatArray = [xhatArray xhat];
    
        % Use FEM-simulation as measurement for haptic feedback
        force_measurement(i) = Force_FEM(i);
        R = R_FEM;  
    
        % Estimation using Kalman filter
        % Predict
        xhat  = Ad*xhat+B*u;               % Priori estimate
        Pmin  = Ad*Pplus*Ad'+Q;             % Priori covariance
        % Update
        K     = Pmin*H'*inv(H*Pmin*H'+R);    % Kalman gain
        xhat  = xhat+K*(force_measurement(i) - H*xhat);          % Posteriori estimate
        Pplus = (eye(2)-K*H)*Pmin;          % Posteriori covariance
    
        %Use empirical data as measurement for haptic feedback
        force_measurement(i) = Force_random_cut_newtons(i);
        R = R_empirical;
        
        % Estimation using Kalman filter
        % Predict
        xhat  = Ad*xhat+B*u;               % Priori estimate
        Pmin  = Ad*Pplus*Ad'+Q;             % Priori covariance
        % Update
        K     = Pmin*H'*inv(H*Pmin*H'+R);    % Kalman gain
        xhat  = xhat+K*(force_measurement(i) - H*xhat);          % Posteriori estimate
        Pplus = (eye(2)-K*H)*Pmin;          % Posteriori covariance
           
end

plot(theta_FEM, xhatArray(1,:));
legend('Empirical Signal', 'FEM-signal', 'KF-estimation');

figure(5)
plot(K);


%% Export empirical data to txt file
%Force_random_cut_newtons = Force_random_cut_newtons';
%empiricalExport(theta_random_cut, Force_random_cut_newtons)
