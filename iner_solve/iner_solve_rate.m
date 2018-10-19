%% 1.Load data
imu = imu_rate;
dt = dt_imu_rate*2;

%% 2.Solve
earth_constant;

n = floor((size(imu,1)-1)/2); %the number of inertial solving
nav = zeros(n,10);
% nav = [timestamp, lat, lon, alt, vn, ve, vd, yaw, pitch, roll]

p = traj(1,2:4); %deg
v = traj(1,5:7); %m/s
att = traj(1,8:10); %deg
pva0 = [p, v, att]; %record initial value

p(1:2) = p(1:2)/180*pi; %rad
att = att/180*pi; %rad
q = angle2quat(att(1), att(2), att(3));
avp = [q, v, p]'; %column vector

for k=1:n
    kj = 2*k+1;
    gyro0 = imu(kj-2, 2:4)'; %rad/s
    gyro1 = imu(kj-1, 2:4)';
    gyro2 = imu(kj  , 2:4)';
    acc0  = imu(kj-2, 5:7)'; %m/s^2
    acc1  = imu(kj-1, 5:7)';
    acc2  = imu(kj  , 5:7)';
    
    avp = RK4(@ins_avp_qn, avp, dt, [gyro0;acc0],[gyro1;acc1],[gyro2;acc2]);
    avp(1:4) = quatnormalize(avp(1:4)')'; %quaternion normalization
    
    nav(k,1) = k*dt;
    nav(k,2:3) = avp(8:9)' /pi*180; %deg
    nav(k,4) = avp(10); %m
    nav(k,5:7) = avp(5:7)'; %m/s
    [r1,r2,r3] = quat2angle(avp(1:4)');
    nav(k,8:10) = [r1,r2,r3] /pi*180; %deg
end
nav = [[0,pva0]; nav];

%% 3.Plot
n = size(nav,1);
t = (0:n-1)*dt;

error = nav(:,2:end) - traj(:,2:end);
error(:,1:2) = error(:,1:2)/180*pi*6378137;
for k=1:n
    if error(k,7)>300
        error(k,7) = error(k,7)-360;
    elseif error(k,7)<-300
        error(k,7) = error(k,7)+360;
    end
    if error(k,9)>300
        error(k,9) = error(k,9)-360;
    elseif error(k,9)<-300
        error(k,9) = error(k,9)+360;
    end
end

figure
subplot(3,3,1)
plot(t, error(:,1))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itL\rm(m)')
grid on

subplot(3,3,4)
plot(t, error(:,2))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\lambda(m)')
grid on

subplot(3,3,7)
plot(t, error(:,3))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\ith\rm(m)')
grid on

subplot(3,3,2)
plot(t, error(:,4))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_n\rm(m/s)')
grid on

subplot(3,3,5)
plot(t, error(:,5))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_e\rm(m/s)')
grid on

subplot(3,3,8)
plot(t, error(:,6))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\itv_d\rm(m/s)')
grid on

subplot(3,3,3)
plot(t, error(:,7))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\psi(\circ)')
grid on

subplot(3,3,6)
plot(t, error(:,8))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\theta(\circ)')
grid on

subplot(3,3,9)
plot(t, error(:,9))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\gamma(\circ)')
grid on