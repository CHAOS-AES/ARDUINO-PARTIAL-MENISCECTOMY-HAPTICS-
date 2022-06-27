
%% Tool-Tissue Interaction Data of Arthroscopic Partial Meniscectomy Punch

% MIT License
% 
% Copyright (c) 2022 Øystein Bjelland NTNU
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%%

clear;
clc;
close all;

%% Import file into Matlab. File should be in txt-format. File must be located in same folder
%filename1 = 'blank_slow_14042021_1758.txt'; 
%filename1 = '2_blank_fast_1759_copy_.txt';
%filename1 = '3_meniscus_slow_1804.txt';
%filename1 = '4_meniscus_slow_1811.txt';
%filename1 = '5_meniscus_fast_1807.txt';
%filename1 = '6_meniscus_fast_1814.txt';
%filename1 = '7_blank_slow_1819.txt';
%filename1 = '8_blank_fast_1821.txt';
filename2 = '9_meniscus_slow_1824.txt';
%filename1 = '10_meniscus_slow_1839.txt';
filename1 = '11_meniscus_fast_1826 - Copy.txt';
%filename1 = '12_meniscus_fast_1842.txt';


A = importdata(filename1);
B = importdata(filename2);


%% We need to rearrange the data on the form iddata(output,input,sampling time)

inputAngle_raw = A(:,4);  %Input angle [deg]
outputFingerForce_raw = A(:,1);   %Output finger force [raw fsr reading]. Set to ":2" for [g]
time_raw = A(:,5);  %Raw time from millis() in Arduino [milliseconds]

time = zeros(length(time_raw),1);
Ts_vect = [];

%% Same for validation data

inputAngle_val = B(:,4);
outputFingerForce_val = B(:,1);
time_val = B(:,5)*10^(-3); %[s]

%% First, we check the sampling time

% Set up a reference sample time to make sure the order of magnitude of our
% sampling time is correct.
Ts_ref = 5*(time_raw(3) - time_raw(2));

for i = 2:length(time_raw)
  
   Ts = time_raw(i) - time_raw(i-1);
   
   % Preventing the time gap between the sampling series to artificially increase sampling time.
   if Ts <  Ts_ref
        Ts_vect = [Ts_vect, Ts];
   end
   
   time(i) = (time_raw(i) - time_raw(1))/1000; %Starting the time vector from zero and converting from ms to s.
   
end

maxTs = max(Ts_vect);
disp('Maximum sample time [ms]: ')
disp(maxTs)

minTs = min(Ts_vect);
disp('Minimum sample time [ms]: ') 
disp(minTs)

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

disp('Our sampling frequency is, Fs [Hz]')
Fs = 1/Ts;
disp(Fs)

%% Potentiometer data contains some noise, we must filter this

inputAngle_filtered = lowpass(inputAngle_raw, 5, Fs);
inputAngle_val_filtered = lowpass(inputAngle_val, 5, Fs);

%% Now, lets structure our data as iddata

FINGER_FORCE_DATA_1 = iddata(outputFingerForce_raw, inputAngle_raw, Ts);
FINGER_FORCE_DATA_FILTERED = iddata(outputFingerForce_raw, inputAngle_filtered, Ts);
FINGER_FORCE_DATA_VALIDATION = iddata(outputFingerForce_val, inputAngle_val_filtered, Ts);

%% Converting raw force data to newtons

outputFingerForce_newtons = zeros(1, length(outputFingerForce_raw));

for i = 1:length(outputFingerForce_raw)    
    % Conversion from calibration data in excel sheet, plus converting from
    % gram to newton
    outputFingerForce_newtons(i) = (3*10^(-7)*(outputFingerForce_raw(i))^3 - 0.0002*(outputFingerForce_raw(i))^2 + 0.7815*outputFingerForce_raw(i))*(10^(-3)*9.81);
end
    
%% Visualize the data before starting system identification

% Raw time domain signals
figure(1)
title('Raw Data')
subplot(2,1,1)
plot(time, inputAngle_raw)
grid on
xlabel('time [s]')
ylabel('Angle (deg)')

subplot(2,1,2)
plot(time, outputFingerForce_raw)
grid on
xlabel('time [s]')
ylabel('Finger Force [no unit]')

% Filtered potentiometer signal compared to raw
figure(2)
plot(time, inputAngle_raw)
grid on
xlabel('time [s]')
ylabel('Angle (deg)')
title('Filtered Potentiometer Data')
hold on
plot(time, inputAngle_filtered)

% Angle vs position
figure(3)
plot(inputAngle_filtered, outputFingerForce_raw, 'r')
grid on
xlabel('Angle (deg)')
ylabel('Finger Force [no unit]')
title('Position vs force')

% FFT of raw potentiometer signal
figure(4)

f = Fs*(0:(length(time)/2))/length(time);
inputAngle_FFT = fft(inputAngle_raw);
P2 = abs(inputAngle_FFT/length(time));
P1 = P2(1:length(time)/2+1);
P1(2:end-1) = 2*P1(2:end-1);
plot(f,10*log10(P1))
xlabel('f (Hz)')
ylabel('amplitude')
title('Fast Fourier Transform')


% PSD plots of raw potentiometer signals
figure(5)
plot(psd(spectrum.periodogram, inputAngle_raw, 'Fs', Fs, 'NFFT', length(inputAngle_raw)))
xlim([0 50])
%plot(psd(periodogram, inputAngle_raw, 'Fs', Fs, 'NFFT', length(inputAngle_raw)))
%[pxx, w] = periodogram(inputAngle_raw);
%plot(w, 10*log10(pxx))

figure(6)
plot(psd(spectrum.welch, inputAngle_raw, 'Fs', Fs, 'NFFT', length(inputAngle_raw)))

% Angle vs position
figure(7)
plot(inputAngle_filtered, outputFingerForce_newtons, 'b')
%grid on
title('Partial Meniscectomy Punch Force Curve');
xlabel('Puncher Angle [degrees]');
ylabel('Finger Force [N]');

figure(8)
title('VALIDATION Data')
subplot(2,1,1)
plot(time_val, inputAngle_val_filtered)
grid on
xlabel('time [s]')
ylabel('Angle (deg)')

subplot(2,1,2)
plot(time_val, outputFingerForce_val)
grid on
xlabel('time [s]')
ylabel('Finger Force [no unit]')