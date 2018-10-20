%% 1.Load data
% data = [timestamp, lat, lon, alt, yaw, pitch, roll]
clc;
dt_data = 0.1;
% data = load('./data/Data_2018-10-19_15-34-24.txt');
% data = load('./data/Data_2018-10-19_20-47-02.txt');
data = load('./data/Data_2018-10-19_21-48-20.txt');

%% 2.Delete repetitive data at the end of the file
while 1
    if data(end,7)==data(end-1,7)
        data(end,:) = [];
    else
        break;
    end
end

%% 3.Generate cmd
n = size(data,1);
cmd = zeros(n,8); %[timestamp, lat, lon, alt, q0,q1,q2,q3]
% cmd(:,1) = (0:n-1)*dt_data;
cmd(:,1) = data(:,1)-data(1,1);
cmd(:,2:4) = data(:,2:4); %position
for k=1:n
    r1 = round(data(k,5),5)/180*pi;
    r2 = round(data(k,6),5)/180*pi;
    r3 = round(data(k,7),5)/180*pi;
    cmd(k,5:8) = angle2quat(r1, r2, r3); %q
end
for k=2:n %make quaternion continue
    if abs(cmd(k,5)-cmd(k-1,5))>1
        cmd(k:end,5:8) = -cmd(k:end,5:8);
    end
end
clearvars -except data cmd
pos0 = data(1,2:4); %deg
att0 = round(data(1,[7,6,5]),5) /180*pi; %rad, [roll, pitch, yaw]

%% 4.Set time parameter
dt_scope = 0.001;
dt_solve = 1e-3;
dt_imu_rate = 10e-3;
dt_imu_delta = 10e-3;
dt_traj = 20e-3;

decimation_imu_rate = dt_imu_rate/dt_solve;
decimation_imu_delta = dt_imu_delta/dt_solve;
decimation_traj = dt_traj/dt_solve;