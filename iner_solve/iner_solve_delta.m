%% 1.Load data
imu = imu_delta;
dt = dt_imu_delta*2;

%% 2.Solve
earth_constant;

n = size(imu,1)/2;
nav = zeros(n,10);

p = traj(1,2:4); %deg
v = traj(1,5:7); %m/s
att = traj(1,8:10); %deg
pva0 = [p, v, att]; %record initial value

att = att/180*pi; %rad
% Cin = dcmecef2ned(p(1), p(2));
% Cnb = angle2dcm(att(1), att(2), att(3));
% Cib = Cnb*Cin;
% q = dcm2quat(Cib);
q = angle2quat(att(1), att(2), att(3));
Cnb = angle2dcm(att(1), att(2), att(3));
p(1:2) = p(1:2)/180*pi; %rad

p_1 = p;
p_2 = p;
v_1 = v;
v_2 = v;

for k=1:n
    kj = 2*k;
    dtheta1 = imu(kj-1,2:4);
    dtheta2 = imu(kj,  2:4);
    dv1 = imu(kj-1,5:7);
    dv2 = imu(kj,  5:7);
    
    lat = (3*p_1(1)-p_2(1))/2;
    h   = (3*p_1(3)-p_2(3))/2;
    vn  = (3*v_1(1)-v_2(1))/2;
    ve  = (3*v_1(2)-v_2(2))/2;
    Rm = (1-f)^2*a / (1-(2-f)*f*sin(lat)^2)^1.5;
    Rn =         a / (1-(2-f)*f*sin(lat)^2)^0.5;
    wien = [w*cos(lat), 0, -w*sin(lat)];
    wenn = [ve/(Rn+h), -vn/(Rm+h), -ve/(Rn+h)*tan(lat)];
    winn = wien+wenn;
    winb = (Cnb*winn')';
    
    %--velocity update--% v = traj(k+1,5:7);
    dtheta1_c = dtheta1 - winb*dt/2;
    dtheta2_c = dtheta2 - winb*dt/2;
    dvc = 1/2*cross(dtheta1_c,dv1)+7/6*cross(dtheta1_c,dv2)-1/6*cross(dtheta2_c,dv1)+1/2*cross(dtheta2_c,dv2);
    dv = (dv1+dv2+dvc)*Cnb - cross(2*wien+wenn,v)*dt + [0,0,9.8]*dt;
    if norm(dv)>1e-10
        v = v + dv;
    end
    
    %--position update--% p = [traj(k+1,2)/180*pi,traj(k+1,3)/180*pi,traj(k+1,4)]; 
    if norm((v+v_1)/2*dt)>1e-10
        p(1) = p(1) + (v(1)+v_1(1))/2 /(Rm+h)*dt;
        p(2) = p(2) + (v(2)+v_1(2))/2 /(Rn+h)*sec(lat)*dt;
        p(3) = p(3) - (v(3)+v_1(3))/2 *dt;
    end
    
    %--attitude update--%
    Phi = dtheta1+dtheta2 + 2/3*cross(dtheta1,dtheta2);
    if norm(Phi)>1e-10
        phi = norm(Phi);
        qc = [cos(phi/2), Phi/phi*sin(phi/2)];
        q = quatmultiply(q, qc);
    end
%     t = k*dt;
%     Cie = [cos(w*t),sin(w*t),0; -sin(w*t),cos(w*t),0; 0,0,1];
%     Cen = dcmecef2ned(p(1)/pi*180, p(2)/pi*180);
%     Cib = quat2dcm(q);
%     Cnb = Cib*Cie'*Cen';
    Cn0b = quat2dcm(q);
    Ci0n0 = dcmecef2ned(p_1(1)/pi*180,p_1(2)/pi*180);
    Ci0i = [cos(w*dt),sin(w*dt),0; -sin(w*dt),cos(w*dt),0; 0,0,1];
    Cin = dcmecef2ned(p(1)/pi*180,p(2)/pi*180);
    Cnb = Cn0b*Ci0n0*Ci0i'*Cin';
    q = dcm2quat(Cnb);
    
    v_2 = v_1;
    v_1 = v;
    p_2 = p_1;
    p_1 = p;
    
    %--store--%
    nav(k,1) = k*dt;
    nav(k,2:3) = p(1:2) /pi*180; %deg
    nav(k,4) = p(3);
    nav(k,5:7) = v;
    [r1,r2,r3] = dcm2angle(Cnb);
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