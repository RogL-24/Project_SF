clear
m = mobiledev;
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
m.MagneticSensorEnabled = 1;
m.OrientationSensorEnabled = 1;
m.logging = 1;
pause(20)

gyro = angvellog(m);
acc = accellog(m);
mag = magfieldlog(m);
orientation = deg2rad(orientlog(m));

[gyro_mean, gyro_cov] = mean_cov(gyro(:,1), gyro(:,2), gyro(:,3));
[acc_mean, acc_cov] = mean_cov(acc(:,1), acc(:,2), acc(:,3));
[mag_mean, mag_cov] = mean_cov(mag(:,1), mag(:,2), mag(:,3));
g0 = [0;0;acc_mean(3,1)];
m0 = [0; sqrt(mag_mean(1,1)^2 + mag_mean(2,1)^2); mag_mean(3,1)];

discardlogs(m);
m.logging = 0;