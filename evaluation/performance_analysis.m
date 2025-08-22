clear;
close all;
clc;

%% Plotting parameters

plot_3D = true;

offset = 72;

%% Load data

% select experiment
exp = "third";

% load mocap data
dMC = load_data_MoCap(sprintf("data\\%sTest.mat",exp));

%dKF = parse_influx_csv("data\query.csv", dMC.timestamp, offset);
dKF = parse_influx_csv(sprintf("data\\%s_estpose.csv",exp), dMC.timestamp, offset);
dRP = parse_influx_csv(sprintf("data\\%s_rawpose.csv",exp), dMC.timestamp, offset);

%% Plot the result of MoCap tracking

if plot_3D
    figure(1); hold on; grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % Plot anchor reference frame (first position)
    x = dMC.anchor.x(1);
    y = dMC.anchor.y(1);
    z = dMC.anchor.z(1);
    plot3(x,y,z, 'b', 'LineWidth', 1.5);
    scale = 200;
    R = eul2rotm([dMC.anchor.Y(1) dMC.anchor.P(1) dMC.anchor.R(1)], 'ZYX'); % MATLAB uses ZYX convention by default
    quiver3(x, y, z, scale*R(1,1), scale*R(2,1), scale*R(3,1), 'r', 'LineWidth', 1.5); % X-axis
    quiver3(x, y, z, scale*R(1,2), scale*R(2,2), scale*R(3,2), 'g', 'LineWidth', 1.5); % Y-axis
    quiver3(x, y, z, scale*R(1,3), scale*R(2,3), scale*R(3,3), 'k', 'LineWidth', 1.5); % Z-axis
    
    % Plot tags trajectories
    plot3(dMC.tag1.x,dMC.tag1.y,dMC.tag1.z, 'b-', 'LineWidth', 1.5)
    plot3(dMC.tag2.x,dMC.tag2.y,dMC.tag2.z, 'r-', 'LineWidth', 1.5)
end

%% Convert to 2D: distance and angles

% tag 1: compute distance and Angle of Arrival
dMC.tag1.d = sqrt((dMC.anchor.x-dMC.tag1.x).^2 + (dMC.anchor.y-dMC.tag1.y).^2) / 10; % [cm] distance from anchor
dMC.tag1.aoa = atan2(dMC.tag1.y-dMC.anchor.y, dMC.tag1.x-dMC.anchor.x) - 0.06;

% convert to 2D position
x_2D = dMC.tag1.d .* sin(dMC.tag1.aoa);
y_2D = dMC.tag1.d .* cos(dMC.tag1.aoa);

% interpolate time intervals for compatibility
d2D.tag1.x = interp1(dMC.t, x_2D, dKF.tag1.t, 'linear', 'extrap');
d2D.tag1.y = interp1(dMC.t, y_2D, dKF.tag1.t, 'linear', 'extrap');
d2D.tag1.t = dMC.t;

% tag 2: compute distance and Angle of Arrival
dMC.tag2.d = sqrt((dMC.anchor.x-dMC.tag2.x).^2 + (dMC.anchor.y-dMC.tag2.y).^2) / 10; % [cm] distance from anchor
dMC.tag2.aoa = atan2(dMC.tag2.y-dMC.anchor.y,dMC.tag2.x-dMC.anchor.x) - 0.06;
 
% convert to 2D position
x_2D = dMC.tag2.d .* sin(dMC.tag2.aoa);
y_2D = dMC.tag2.d .* cos(dMC.tag2.aoa);

d2D.tag2.x = interp1(dMC.t, x_2D, dKF.tag2.t, 'linear', 'extrap');
d2D.tag2.y = interp1(dMC.t, y_2D, dKF.tag2.t, 'linear', 'extrap');
d2D.tag2.t = dMC.t;

%% Plot them together

st = 280; % start time
et = 468; % stop time

% Kalman Filter estimates against Ground Truth
figure(2); hold on; grid on; axis equal; xlim([-170,130])
%title("Comparison between estimated positions (KF) and ground truth (GT)")
xlabel('x [cm]'); ylabel('y [cm]');

plot(dKF.tag1.x(st:et),dKF.tag1.y(st:et), 'b-',LineWidth=1.5, LineStyle='-')
plot(dKF.tag2.x(st:et),dKF.tag2.y(st:et), 'r-',LineWidth=1.5, LineStyle='-')

plot(d2D.tag1.x(st:et),d2D.tag1.y(st:et), 'b-',LineWidth=1.5, LineStyle='--')
plot(d2D.tag2.x(st:et),d2D.tag2.y(st:et), 'r-',LineWidth=1.5, LineStyle='--')

plot(0,0, 'k.', 'Marker','o', 'MarkerSize',14)

legend(["tag 1 KF", "tag 2 KF", "tag 1 GT", "tag 2 GT", "anchor"])

% Raw Position against Ground Truth
figure(3); hold on; grid on; axis equal; xlim([-170,130])
%title("Comparison between raw positions (RP) and ground truth (GT)")
xlabel('x [cm]'); ylabel('y [cm]');
plot(dRP.tag1.x(st:et),dRP.tag1.y(st:et), 'b-',LineWidth=0.8, LineStyle='-')
plot(dRP.tag2.x(st:et),dRP.tag2.y(st:et), 'r-',LineWidth=0.8, LineStyle='-')

plot(d2D.tag1.x(st:et),d2D.tag1.y(st:et), 'b-',LineWidth=1.5, LineStyle='--')
plot(d2D.tag2.x(st:et),d2D.tag2.y(st:et), 'r-',LineWidth=1.5, LineStyle='--')

plot(0,0, 'k.', 'Marker','o', 'MarkerSize',14)

legend(["tag 1 RP", "tag 2 RP", "tag 1 GT", "tag 2 GT", "anchor"])

%% Tracking error

% Kalman Filter estimates against Ground Truth
dx = dKF.tag1.x(st:et) - d2D.tag1.x(st:et);
dy = dKF.tag1.y(st:et) - d2D.tag1.y(st:et);
KF_track_err_1 = vecnorm([dx,dy],2,2);

dx = dKF.tag2.x(st:et) - d2D.tag2.x(st:et);
dy = dKF.tag2.y(st:et) - d2D.tag2.y(st:et);
KF_track_err_2 = vecnorm([dx,dy],2,2);

figure(4); clf;
% Tag 1 error
subplot(2,1,1); hold on; grid on;
plot(dKF.tag1.t(st:et), KF_track_err_1, 'b-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Error [cm]');
title('Tag 1');
% Tag 2 error
subplot(2,1,2); hold on; grid on;
plot(dKF.tag2.t(st:et), KF_track_err_2, 'r-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Error [cm]');
title('Tag 2');


% Raw Position against Ground Truth
dx = dRP.tag1.x(st:et) - d2D.tag1.x(st:et);
dy = dRP.tag1.y(st:et) - d2D.tag1.y(st:et);
RP_track_err_1 = vecnorm([dx,dy],2,2);

dx = dRP.tag2.x(st:et) - d2D.tag2.x(st:et);
dy = dRP.tag2.y(st:et) - d2D.tag2.y(st:et);
RP_track_err_2 = vecnorm([dx,dy],2,2);

figure(5); clf;
% Tag 1 error
subplot(2,1,1); hold on; grid on;
plot(dRP.tag1.t(st:et), RP_track_err_1, 'b-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Error [cm]');
title('Tag 1');
% Tag 2 error
subplot(2,1,2); hold on; grid on;
plot(dRP.tag2.t(st:et), RP_track_err_2, 'r-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Error [cm]');
title('Tag 2');

% print results
fprintf("KF estimation\n")
fprintf("Average tracking error:\t\t\t\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", mean(KF_track_err_1), mean(KF_track_err_2))
fprintf("Root Mean Square tracking error:\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", sqrt(mean(KF_track_err_1.^2)), sqrt(mean(KF_track_err_2.^2)))
fprintf("Standard deviation of tracking error:\t tag 1 = %0.2d \t tag 2 = %0.2d\n", std(KF_track_err_1), std(KF_track_err_2))
fprintf("95%c of tracking error:\t\t\t\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", '%', prctile(KF_track_err_1, 95), prctile(KF_track_err_2, 95))


fprintf("\nRaw Measurements\n")
fprintf("Average tracking error:\t\t\t\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", mean(RP_track_err_1), mean(RP_track_err_2))
fprintf("Root Mean Square tracking error:\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", sqrt(mean(RP_track_err_1.^2)), sqrt(mean(RP_track_err_2.^2)))
fprintf("Standard deviation of tracking error:\t tag 1 = %0.2d \t tag 2 = %0.2d\n", std(RP_track_err_1), std(RP_track_err_2))
fprintf("95%c of tracking error:\t\t\t\t\t tag 1 = %0.2d \t tag 2 = %0.2d\n", '%', prctile(RP_track_err_1, 95), prctile(RP_track_err_2, 95))

%% RSSI

figure(6); clf;
% Tag 1 error
subplot(2,1,1); hold on; grid on;
plot(dKF.tag1.t(st:et), dKF.tag1.rssi(st:et), 'b-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('RSSI [dBm]');
title('Tag 1');
% Tag 2 error
subplot(2,1,2); hold on; grid on;
plot(dKF.tag2.t(st:et), dKF.tag2.rssi(st:et), 'r-', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('RSSI [dBm]');
title('Tag 2');


%% Functions

function data = load_data_MoCap(filename)
    tmp = load(filename);
    fn = fieldnames(tmp);
    mat = tmp.(fn{1});
    
    data = struct;
    data.timestamp = parse_starttime(mat.Timestamp);
    dt = 1/mat.FrameRate;
    data.t = (mat.StartFrame:mat.Frames+mat.StartFrame-1)*dt;
    
    for b = 1:mat.RigidBodies.Bodies
        name =  string(mat.RigidBodies.Name(b));
            
        data.(name).x = squeeze(mat.RigidBodies.Positions(b,1,:));
        data.(name).y = squeeze(mat.RigidBodies.Positions(b,2,:));
        data.(name).z = squeeze(mat.RigidBodies.Positions(b,3,:));
    
        data.(name).R = squeeze(mat.RigidBodies.RPYs(b,1,:));
        data.(name).P = squeeze(mat.RigidBodies.RPYs(b,2,:));
        data.(name).Y = squeeze(mat.RigidBodies.RPYs(b,3,:));
    end
end

function t0 = parse_starttime(starttime_str)
    % Extract only the datetime portion before the tab
    tokens = split(starttime_str, sprintf('\t'));
    dt_str = strtrim(tokens{1});
    
    % Convert to datetime
    t0 = datetime(dt_str, ...
        'InputFormat','yyyy-MM-dd, HH:mm:ss.SSS', ...
        'TimeZone','UTC');
end

function data = parse_influx_csv(filename, starttime, offset)
    % Read file, skip commented metadata lines
    opts = detectImportOptions(filename, 'NumHeaderLines', 3);  
    T = readtable(filename, opts);

    % Keep only the columns we need
    time = datetime(T.x_time, 'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSSX', 'TimeZone','UTC');
    temp.t    = seconds(time - starttime) + 2*3600 + offset;
    
    temp.ag   = T.agent;
    temp.rssi = T.rssi;
    temp.x    = T.x;
    temp.y    = T.y;

    % initialize empty struct
    data.tag1.t = [];
    data.tag1.x = [];
    data.tag1.y = [];
    data.tag1.rssi = [];
    data.tag2.t = [];
    data.tag2.x = [];
    data.tag2.y = [];
    data.tag2.rssi = [];
    for i = 1:length(temp.t)
        if temp.ag(i) == "agent_1"
            data.tag1.t = [data.tag1.t; temp.t(i)];
            data.tag1.x = [data.tag1.x; temp.x(i)];
            data.tag1.y = [data.tag1.y; temp.y(i)];
            data.tag1.rssi = [data.tag1.rssi; temp.rssi(i)];
        elseif temp.ag(i) == "agent_2"
            data.tag2.t = [data.tag2.t; temp.t(i)];
            data.tag2.x = [data.tag2.x; temp.x(i)];
            data.tag2.y = [data.tag2.y; temp.y(i)];
            data.tag2.rssi = [data.tag2.rssi; temp.rssi(i)];
        end
    end
end