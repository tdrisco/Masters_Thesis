%Read in all trial data and display main graph

clear
clc
close all

%% Navigate to and open all trial data and save in large data structure

subjects_R = {'AB01', 'AB02', 'AB03', 'AB04', 'AB05', 'AB06'};
inclines_R = {'0', '5', '10', 'neg5', 'neg10'};
inclines2_R = {'pos0', 'pos5', 'pos10', 'neg5', 'neg10'};
speeds_R = {'slow', 'normal', 'fast', 'switch'};


for i=1:1:length(subjects_R)
    for j = 1:1:length(inclines_R)
        for k = 1:1:length(speeds_R)
            %Read in all 120 tables
            filename = sprintf('%s/%s_%s/%s_%s_%s.csv',subjects_R{i},subjects_R{i},inclines_R{j},subjects_R{i},speeds_R{k},inclines_R{j});
            Data.(subjects_R{i}).(inclines2_R{j}).(speeds_R{k}) = readtable(filename);
        end
    end
end

%% Pull out specific data for evaluation
subjects_P = {'AB01'};
inclines_P = {'pos0'};
speeds_P = {'normal'};
count = 1;

for i=1:1:length(subjects_P)
    for j = 1:1:length(inclines_P)
        for k = 1:1:length(speeds_P)
            time{count} = Data.(subjects_P{i}).(inclines_P{j}).(speeds_P{k}).Time_Sec_;
            phase{count} = Data.(subjects_P{i}).(inclines_P{j}).(speeds_P{k}).PhaseVariable;
            frequency{count} = Data.(subjects_P{i}).(inclines_P{j}).(speeds_P{k}).StepFrequency_Hz_;
            thighAngle{count} = Data.(subjects_P{i}).(inclines_P{j}).(speeds_P{k}).PitchAngle_Deg_;
            thighAngleVel{count} = Data.(subjects_P{i}).(inclines_P{j}).(speeds_P{k}).AngularVelocityZ_deg_s_;
            count = count + 1;
        end
    end
end

%% Setup CDS Variables
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

T = 1/100; %Period

transition = 0;
phase_shift = 0;
phase_shift_final = 0;
buf_count = 1;
buf_loc = 1;

filtered_data = thighAngleVel{1};
Thigh_angle_input = thighAngle{1}
Ang = 0;
first_step = 1;
step_count = 1;
transition_count = 1;

points = 1:1:length(filtered_data);

%% Feedback Loop Process for Demo
for i = 1:1:length(points)
    
    prev_phi = phi;
    prev_Ang = Ang;

    %Obtain current data reading (input)
    y = filtered_data(i);
    Ang = Thigh_angle_input(i)

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

    prev_step(step_count) = phi;
    step_count = step_count + 1;

    if(phi < prev_phi)
            transition = 1;
    end
    if(transition)
       if(Ang < prev_Ang)
           transition = 0;
           phase_shift_final = phase_shift
           phase_shift = 0;
       else
           phase_shift = phase_shift + 1;
       end
        
    end

    %Save variables for plot
    %---------------------------------------
    phase_CDS(i) = phi;
    frequency_CDS(i) = w/ (2*pi());
    output(1,i) = Ang;
    
    if(buf_count == 100)
        phi_buffer(1:50) = phi_buffer(50:99);
        buf_count = 51;
    end
    phi_buffer(buf_count) = phi;
    buf_count = buf_count + 1;
    buf_loc = buf_loc + 1;

    if(phase_shift_final == 0)
        output(2,i) = 0;
    else
        output(2,i) = phi_buffer(buf_count - phase_shift_final);
    end
    %---------------------------------------
end

figure('Color','W');
plot((points/100),phase_CDS)
hold on
plot(points/100,Thigh_angle_input)
title('Phase VS. Time');
xlabel('Time [s] Seconds')
ylabel('Phase CDS')
grid on

figure('Color','W');
plot((points/100),phase_CDS)
hold on
plot(points/100,output(1,:))
plot((points/100),output(2,:),':')
title('Phase VS. Time');
xlabel('Time [s] Seconds')
ylabel('Phase CDS')
grid on

figure('Color','W');
subplot(2,1,1)
plot((points(1000:2000)-1000)/100,output(1,1000:2000),'Color',[0 0.4470 0.7410])
ylabel('Thigh Angle Degrees [Deg]')
subplot(2,1,2)
plot(((points(1000:2000)-1000)/100),normalize(phase_CDS(1000:2000),"range"), 'Color',[0.8500 0.3250 0.0980] )
hold on
plot(((points(1000:2000)-1000)/100),normalize(output(2,1000:2000),"range"),'--', 'Color', [0.4660 0.6740 0.1880])
ylabel('Normalized Phase Variable')
legend('Phase', 'Shifted Phase', 'Location','northeast')
xlabel('Time [s] Seconds')

figure('Color','W');
plot((points(1:500)/100),normalize(phase_CDS(1:500),"range"), 'Color',[0.8500 0.3250 0.0980] )
hold on
plot(((points(1:500))/100),normalize(output(2,1:500),"range"),'--', 'Color', [0.4660 0.6740 0.1880])
xlabel('Time [s] Seconds')
ylabel('Normalized Phase Variable')
legend('Phase', 'Shifted Phase', 'Location','northeast')
grid on

