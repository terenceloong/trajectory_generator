%% 1.Load data
clc;
dt_data = 0.1;
% data = load('./data/Data_2018-10-19_15-34-24.txt');
% data = load('./data/Data_2018-10-19_20-47-02.txt');
data = load('./data/Data_2018-10-19_21-48-20.txt');
% data = [timestamp, lat, lon, alt, yaw, pitch, roll]

%% 2.Delete repetitive data at the end of the file
while 1
    if data(end,7)==data(end-1,7)
        data(end,:) = [];
    else
        break;
    end
end

%% 3.Make yaw continue
n = size(data,1);
for k=2:n
    if data(k,5)-data(k-1,5)<-180
        data(k:end,5) = data(k:end,5)+360;
    elseif data(k,5)-data(k-1,5)>180
        data(k:end,5) = data(k:end,5)-360;
    end
end

%% 4.Generate cmd
cmd = [data, zeros(n,4)];
% cmd(:,1) = (0:n-1)*dt_data;
cmd(:,1) = cmd(:,1)-cmd(1,1);
for k=1:n
    q = angle2quat(cmd(k,5)/180*pi, cmd(k,6)/180*pi, cmd(k,7)/180*pi);
    theta = 2*acos(q(1));
    cmd(k,8) = theta;
    cmd(k,9) = q(2)/sin(theta/2);
    cmd(k,10) = q(3)/sin(theta/2);
    cmd(k,11) = q(4)/sin(theta/2);
end
clearvars -except data cmd

%% 5.Set time parameter
dt_scope = 0.001;
dt_solve = 1e-3;
dt_imu_rate = 10e-3;
dt_imu_delta = 10e-3;
dt_traj = 20e-3;

decimation_imu_rate = dt_imu_rate/dt_solve;
decimation_imu_delta = dt_imu_delta/dt_solve;
decimation_traj = dt_traj/dt_solve;