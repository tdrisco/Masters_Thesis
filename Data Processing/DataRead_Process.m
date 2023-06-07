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

%% Plot data that we want to look at further
subjects_P = {'AB01'};
inclines_P = {'pos0','pos5','pos10'};
speeds_P = {'slow', 'normal', 'fast', 'switch'};
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

% points = 1:length(phase{1});
% points = points/100;
% 
% figure('Color','W');
% plot(points,phase{1})
% hold on
% plot(points,frequency{1})
% plot(points,phase{3})
% plot(points,frequency{3})
% title('Phase VS. Time');
% xlabel('Time [s] Seconds')
% ylabel('Phase CDS')
% grid on
% 
% figure('Color','W');
% plot(points,phase{1})
% hold on
% plot(points,frequency{1},'k:')
% title('Phase VS. Time');
% xlabel('Time [s] Seconds')
% ylabel('Phase CDS')
% grid on

%%find points of interest in phase
for jj = 1:1:length(phase)
    phase_points = 1;
    ang_points = 1;
    count = 1;
    count2 = 1;
    for i=1:1:length(phase{jj})-1
        phase_breakdown{phase_points}(count) = phase{jj}(i);
        AngVel_breakdown{phase_points}(count) = thighAngleVel{jj}(i);
        Ang_breakdown{phase_points}(count) = thighAngle{jj}(i);
        Ang2_breakdown{ang_points}(count2) = thighAngle{jj}(i);
        count = count + 1;
        count2 = count2 + 1;
        if(phase{jj}(i+1) < phase{jj}(i))
            transition(phase_points) = i;
            phase_points = phase_points + 1;
            count = 1;
        end

        if(thighAngle{jj}(i+1) < thighAngle{jj}(i) && count2 > count)
            transition2(ang_points) = i;
            ang_points = ang_points + 1;
            count2 = 1;
        end

    end
    
    for i=1:1:phase_points
        phase_sizes(i) = length(phase_breakdown{i});
    end

    for i=1:1:ang_points
        Ang2_sizes(i) = length(Ang2_breakdown{i});
    end
    
    max_length = max(phase_sizes(2:end-1));
    max_length2 = max(Ang2_sizes(2:end-1));
    count = 1;
    
    for i=1:1:phase_points
        if(max_length-phase_sizes(i) < 15)
            Outliers(count) = i;
            count = count + 1;
        end
    end
    
    for i=1:1:length(Outliers)
        phase_segments{i} = phase_breakdown{Outliers(i)};
        AngVel_segments{i} = AngVel_breakdown{Outliers(i)};
        Ang_segments{i} = Ang_breakdown{Outliers(i)};
        Ang2_segments{i} = Ang2_breakdown{Outliers(i)};
    end
    
    for i = 1:1:length(phase_segments)
        xnew = linspace(0.01,length(phase_segments{i})/100,max_length);
    
        xold = 1:length(phase_segments{i});
        xold = xold/100;
    
        phase_interp{i} = interp1(xold,phase_segments{i},xnew,'linear');
        phase_interp_mat(i,:) = phase_interp{i};

        AngVel_interp{i} = interp1(xold,AngVel_segments{i},xnew,'linear');
        AngVel_interp_mat(i,:) = AngVel_interp{i};

        Ang_interp{i} = interp1(xold,Ang_segments{i},xnew,'linear');
        Ang_interp_mat(i,:) = Ang_interp{i};

        xnew = linspace(0.01,length(Ang2_segments{i})/100,max_length2);
    
        xold = 1:length(Ang2_segments{i});
        xold = xold/100;

        Ang2_interp{i} = interp1(xold,Ang2_segments{i},xnew,'linear');
        Ang2_interp_mat(i,:) = Ang2_interp{i};
    end
    
    phase_mean{jj} = mean(phase_interp_mat);
    phase_mean_norm{jj} = normalize(phase_mean{jj},"range");

    AngVel_mean{jj} = mean(AngVel_interp_mat);
    Ang_mean{jj} = mean(Ang_interp_mat);
    Ang2_mean{jj} = mean(Ang2_interp_mat);
    
    points = 1:length(phase_mean{jj});
    points = points/100;
    phase_points_array{jj} = points;
    %points = rescale(points,0,100);

    points2 = 1:length(Ang2_mean{jj});
    points2 = points2/100;
    Ang2_points_array{jj} = points2;

    clear phase_breakdown
    clear transition
    clear phase_sizes
    clear Outliers
    clear phase_segments
    clear phase_interp
    clear phase_interp_mat
    
    clear AngVel_breakdown
    clear AngVel_segments
    clear AngVel_interp
    clear AngVel_interp_mat

    clear Ang_breakdown
    clear Ang_segments
    clear Ang_interp
    clear Ang_interp_mat

    clear Ang2_breakdown
    clear transition2
    clear Ang2_sizes
    clear Ang2_segments
    clear Ang2_interp
    clear Ang2_interp_mat

end

figure('Color','W');
plot(phase_points_array{1},phase_mean_norm{1})
hold on
plot(phase_points_array{2},phase_mean_norm{2})
plot(phase_points_array{3},phase_mean_norm{3})
legend('0.67 m/s', '1.0 m/s', '1.35 m/s', 'Location','northwest')
title('Phase VS. Time AB01 0 Degrees');
xlabel('Time [s] Seconds')
ylabel('Phase CDS')
grid on

figure('Color','W');
plot(phase_points_array{1},AngVel_mean{1})
hold on
plot(phase_points_array{2},AngVel_mean{2})
plot(phase_points_array{3},AngVel_mean{3})
legend('0.67 m/s', '1.0 m/s', '1.35 m/s', 'Location','northwest')
title('Thigh Angular Velocity VS. Time AB01 0 Degrees');
xlabel('Time [s] Seconds')
ylabel('Degrees per Second [Deg/s]')
grid on

figure('Color','W');
plot(phase_points_array{1},Ang_mean{1})
hold on
plot(phase_points_array{2},Ang_mean{2})
plot(phase_points_array{3},Ang_mean{3})
legend('0.67 m/s', '1.0 m/s', '1.35 m/s', 'Location','northeast')
title('Thigh Angle VS. Time AB01 0 Degrees');
xlabel('Time [s] Seconds')
ylabel('Degrees [Deg]')
grid on

figure('Color','W');
plot(Ang2_points_array{1},Ang2_mean{1})
hold on
plot(Ang2_points_array{2},Ang2_mean{2})
plot(Ang2_points_array{3},Ang2_mean{3})
plot(phase_points_array{1},phase_mean_norm{1})
plot(phase_points_array{2},phase_mean_norm{2})
plot(phase_points_array{3},phase_mean_norm{3})
legend('0.67 m/s', '1.0 m/s', '1.35 m/s', '0.67 m/s phase', '1.0 m/s phase', '1.35 m/s phase', 'Location','southeast')
title('Thigh Angle2 VS. Time AB01 0 Degrees');
xlabel('Time [s] Seconds')
ylabel('Degrees [Deg]')
grid on


% figure('Color','W');
% plot(phase_points_array{5},phase_mean_norm{5})
% hold on
% plot(phase_points_array{6},phase_mean_norm{6})
% plot(phase_points_array{7},phase_mean_norm{7})
% legend('0.67 m/s', '1.0 m/s', '1.35 m/s', 'Location','northwest')
% title('Phase VS. Time AB01 5 Degrees');
% xlabel('Time [s] Seconds')
% ylabel('Phase CDS')
% grid on
% 
% figure('Color','W');
% plot(phase_points_array{9},phase_mean_norm{9})
% hold on
% plot(phase_points_array{10},phase_mean_norm{10})
% plot(phase_points_array{11},phase_mean_norm{11})
% legend('0.67 m/s', '1.0 m/s', '1.35 m/s', 'Location','northwest')
% title('Phase VS. Time AB01 10 Degrees');
% xlabel('Time [s] Seconds')
% ylabel('Phase CDS')
% grid on

