%% Clean up workspace
clearvars -except GyroTestTim01 GyroTestTim02 GyroTestTim03 GyroTestTim04 GyroTestTim05 CDSTestTim08

clc
close all

%% Setup Variables for CDS

T = 1/100; %Period

filtered_data = CDSTestTim08.AngularVelocityZdegs;
Thigh_angle = CDSTestTim08.PitchAngleDeg;

points = 1:1:length(filtered_data);


%% Establish all variables for the model

M = 7; % # of Fourier series components
eta = 1; %Learning Coefficient
mu = 0.1; %Coupling Constant

estimate = 0; %Predicted state variable
y = 0; %Measured state variable
error = 0; %Error y - y_hat

w = 2*pi()*2/5; %Frequency Value
phi = 0; %Phase Value

ac = zeros(M,1); %Fourier Series Coefficient
bc = zeros(M,1); %Fourier Series Coefficient

c = 0; %Summation Counter


%% Feedback Loop Process for Demo
for i = 1:1:length(points)
    
    %Obtain current data reading (input)
    y = filtered_data(i);

    %Calculate the state variable prediction
    estimate = 0;
    %---------------------------------------
    for c= 0:1:M-1
        estimate = estimate + ac(c+1)*cos(c*phi) + bc(c+1)*sin(c*phi);
    end
    %---------------------------------------

    %Update Variables
    %---------------------------------------
    error = y - estimate;

    percent_error = abs((estimate-y)/y)*100;

    w_curr = w;
    w = abs(w_curr - T*mu*error*sin(phi));

    for c= 0:1:M-1
        ac(c+1) = ac(c+1) + eta * T * cos(c*phi)*error;
        bc(c+1) = bc(c+1) + eta * T * sin(c*phi)*error;
    end
    %---------------------------------------
    
    phi_curr = phi;
    phi_next = mod(phi_curr + T*(w_curr - mu*error*sin(phi_curr)), 2*pi());

    if(mod((phi_next - phi_curr),2*pi()) > 0.5*pi())
        phi = phi_curr;
    else
        phi = phi_next;
    end

    %Save variables for plot
    %---------------------------------------
    y_hat_Save(i) = estimate;
    phase_CDS_Save(i) = phi;
    frequency_Save(i) = w/ (2*pi());
    error_Save(i) = error;
    ac_save(i) = ac(1);
    bc_save(i) = bc(1);
    %---------------------------------------
end

phase = atan2(CDSTestTim08.AngularVelocityZdegs,-Thigh_angle);

%% Plot CDS Outputs
% figure('Color','W');
% plot(points,phase_CDS_Save)
% hold on
% plot(points,CDSTestTim08.PhaseVariable,'k:')
% title('Phase VS. Time');
% xlabel('Time [s] Seconds')
% ylabel('Phase CDS')
% grid on

figure('Color','W');
plot(points,frequency_Save,'r-')
hold on
plot(points,CDSTestTim08.StepFrequencyHz,'k:')
title('Frequency VS. Time');
xlabel('Time [s] Seconds')
ylabel('Frequecny CDS')
grid on

figure('Color','W');
plot(points,y_hat_Save)
hold on
plot(points,phase_CDS_Save)
plot(points,phase)
title('Y Predicted VS. Time with Phase Overlayed');
xlabel('Time [s] Seconds')
ylabel('Predicted Angle')
grid on

figure('Color','W');
plot(points,Thigh_angle)
hold on
plot(points,phase_CDS_Save)
plot(points,phase)
title('Thigh Angle VS. Time with Phase Overlayed');
xlabel('Time [s] Seconds')
ylabel('Degrees')
grid on

% figure('Color','W');
% plot(points,filtered_data,'r-')
% hold on
% plot(points,y_hat_Save,'b:')
% title('Y Measured VS. Time');
% xlabel('Time [s] Seconds')
% ylabel('Measured Angle')
% grid on
% 
% figure('Color','W');
% plot(points,error_Save)
% title('Error VS. Time');
% xlabel('Time [s] Seconds')
% ylabel('Error')
% grid on

%% Traditional Phase Calculations
figure('Color','W');
plot(points,phase)
title('Phase VS. Time Traditional Calculation');
xlabel('Time [s] Seconds')
ylabel('Phase (atan2)')
grid on