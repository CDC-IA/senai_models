# Differential drive kinematics model
wheel_radius = 0.0625;
wheel_track = 0.445208;

# initial conditions
x_0 = 0.0;
y_0 = 0.0;
theta_0 = 0.0;

# time vector and initializing x y and theta vectors
t = linspace(0,3,60);
dt = t(2)-t(1);

x = zeros(1, length(t));
y = zeros(1, length(t));
theta = zeros(1, length(t));

x(1) = x_0;
y(1) = y_0;
theta(1) = theta_0;

# Joints functions

omega_desired = 2*pi;

omega_l = [(ones(1, length(t)/3) * omega_desired/2.0) (ones(1, length(t)/3) * omega_desired) (ones(1, length(t)/3) * omega_desired)];
omega_r = [(ones(1, length(t)/3) * omega_desired) (ones(1, length(t)/3) * omega_desired/2.0) (ones(1, length(t)/3) * omega_desired)];


# Forward kinematics

velocities = [
              zeros(1, length(t));
              zeros(1, length(t));
              zeros(1, length(t))
             ];

for i=2:length(t)
  omega = [omega_r(i-1); omega_l(i-1)];

  velocities(:,i) = [
    wheel_radius/2.0 * cos(theta(i-1)), wheel_radius/2.0 * cos(theta(i-1));
    wheel_radius/2.0 * sin(theta(i-1)), wheel_radius/2.0 * sin(theta(i-1));
    wheel_radius/wheel_track, -wheel_radius/wheel_track
  ] * omega;

  x(i) = x(i-1) + velocities(1,i)*dt;
  y(i) = y(i-1) + velocities(2,i)*dt;
  theta(i) = theta(i-1) + velocities(3,i)*dt;
end


# Inverse Kinematics

vel_x_desired = velocities(1,:)
vel_y_desired = velocities(2,:);
theta_desired = atan2(vel_y_desired,vel_x_desired)
omega_z_desired = velocities(3,:);

joints = [
          zeros(1,length(t));
          zeros(1,length(t))
          ];

for i=1:length(t)
  joints(:,i) = [
    (1/(wheel_radius*cos(theta_desired(i)))), (wheel_track/(2.0*wheel_radius)); # RIGHT JOINT
    (1/(wheel_radius*cos(theta_desired(i)))), -(wheel_track/(2.0*wheel_radius)) # LEFT JOINT
  ]* [vel_x_desired(i); omega_z_desired(i)]
end

figure(1);
plot(t,joints(1,:),'b', t,joints(2,:),'r');
title("Joints");
xlabel('t');
ylabel('angular velocity');
hold on;

# calculate pose

x_estimate = zeros(1, length(t));
y_estimate = zeros(1, length(t));
theta_estimate = zeros(1, length(t));

x_estimate(1) = x_0;
y_estimate(1) = y_0;
theta_estimate(1) = theta_0;


for i=2:length(t)

  vels = [
    wheel_radius/2.0 * cos(theta_estimate(i-1)), wheel_radius/2.0 * cos(theta_estimate(i-1));
    wheel_radius/2.0 * sin(theta_estimate(i-1)), wheel_radius/2.0 * sin(theta_estimate(i-1));
    wheel_radius/wheel_track, -wheel_radius/wheel_track
  ] * joints(:,i-1);

  x_estimate(i) = x_estimate(i-1) + vels(1)*dt;
  y_estimate(i) = y_estimate(i-1) + vels(2)*dt;
  theta_estimate(i) = theta_estimate(i-1) + vels(3)*dt;
end

figure(2);
#plot(x_estimate,y_estimate,'g');
plot(x,y,'k--', x_estimate,y_estimate,'g++');
title("Space moving");
xlabel('x');
ylabel('y');
hold on;
