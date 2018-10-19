%% 1.Load data
imu = imu_rate;
dt = dt_imu_rate*2;

%% 2.Solve
earth_constant;

n = floor((size(imu,1)-1)/2);
nav = zeros(n,3);

att = traj(1,8:10); %deg
att0 = att;

att = att/180*pi; %rad
Cin = dcmecef2ned(traj(1,2), traj(1,3));
Cnb = angle2dcm(att(1), att(2), att(3));
Cib = Cnb*Cin;
q = dcm2quat(Cib)';

for k=1:n
    kj = 2*k+1;
    gyro0 = imu(kj-2, 2:4)'; %rad/s
    gyro1 = imu(kj-1, 2:4)';
    gyro2 = imu(kj  , 2:4)';
    
    q = RK4(@fun_dq, q, dt, gyro0,gyro1,gyro2);
    q = quatnormalize(q')'; %quaternion normalization
    
    t = k*dt;
    Cie = [cos(w*t),sin(w*t),0; -sin(w*t),cos(w*t),0; 0,0,1];
    Cen = dcmecef2ned(traj(k+1,2), traj(k+1,3));
    Cib = quat2dcm(q');
    Cnb = Cib*Cie'*Cen';

    [r1,r2,r3] = dcm2angle(Cnb);
    nav(k,:) = [r1,r2,r3] /pi*180; %deg
end
nav = [att0; nav];

%% 3.Plot
n = size(nav,1);
t = (0:n-1)*dt;
error = nav - traj(:,8:10);
for k=1:n
    if error(k,1)>300
        error(k,1) = error(k,1)-360;
    elseif error(k,1)<-300
        error(k,1) = error(k,1)+360;
    end
    if error(k,3)>300
        error(k,3) = error(k,3)-360;
    elseif error(k,3)<-300
        error(k,3) = error(k,3)+360;
    end
end

figure
subplot(3,1,1)
plot(t, error(:,1))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\psi(\circ)')
grid on

subplot(3,1,2)
plot(t, error(:,2))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\theta(\circ)')
grid on

subplot(3,1,3)
plot(t, error(:,3))
set(gca, 'xlim', [t(1),t(end)])
ylabel('\delta\gamma(\circ)')
grid on