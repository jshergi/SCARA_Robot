%hello

j1limitUpper = 150;
j1limitLower = -150;
j2limitUpper = 100;
j2limitLower = -100;
j3limitUpper = -100;
j3limitLower = -200;
j4limitUpper = 160;
j4limitLower = -160;

vMaxR = 150.0;
vMinR = -150.0;
vMaxP = 50.0;
vMinP = -50.0;

aMaxR = 600.0;
aMinR = -600.0;
aMaxP = 200.0;
aMinP = -200;

posTable = readtable("pos.csv");
velTable = readtable("vel.csv");
accTable = readtable("acc.csv");
viaTable = readtable("via.csv");


posMatrix = table2array(posTable);
velMatrix = table2array(velTable);
accMatrix = table2array(accTable);
viaMatrix = table2array(viaTable);
timeDivision = viaMatrix(5, 1);

noOfSamples = height(posMatrix); %Assuming all are the same height


%Position plots
figure(1);

subplot(1, 4, 1);
plot(posMatrix(:, 1), posMatrix(:, 2));
hold on;
plot(posMatrix(:, 1), ones(noOfSamples) * j1limitUpper);
plot(posMatrix(:, 1), ones(noOfSamples) * j1limitLower);

%Via points
plot(0, viaMatrix(1, 1), 'ro');
plot(timeDivision, viaMatrix(1, 2), 'ro');
plot(timeDivision*2, viaMatrix(1, 3), 'ro');
plot(timeDivision*3, viaMatrix(1, 4), 'ro');
plot(timeDivision*4, viaMatrix(1, 5), 'ro');

hold off;
title("Position of joint 1");

subplot(1, 4, 2);
plot(posMatrix(:, 1), posMatrix(:, 3));
hold on;
plot(posMatrix(:, 1), ones(noOfSamples) * j2limitUpper);
plot(posMatrix(:, 1), ones(noOfSamples) * j2limitLower);

%Via points
plot(0, viaMatrix(2, 1), 'ro');
plot(timeDivision, viaMatrix(2, 2), 'ro');
plot(timeDivision*2, viaMatrix(2, 3), 'ro');
plot(timeDivision*3, viaMatrix(2, 4), 'ro');
plot(timeDivision*4, viaMatrix(2, 5), 'ro');
hold off;
title("Position of joint 2");

subplot(1, 4, 3);
plot(posMatrix(:, 1), posMatrix(:, 4));
hold on;
plot(posMatrix(:, 1), ones(noOfSamples) * j3limitUpper);
plot(posMatrix(:, 1), ones(noOfSamples) * j3limitLower);

%Via points
plot(0, viaMatrix(3, 1), 'ro');
plot(timeDivision, viaMatrix(3, 2), 'ro');
plot(timeDivision*2, viaMatrix(3, 3), 'ro');
plot(timeDivision*3, viaMatrix(3, 4), 'ro');
plot(timeDivision*4, viaMatrix(3, 5), 'ro');
hold off;
title("Position of joint 3");

subplot(1, 4, 4);
plot(posMatrix(:, 1), posMatrix(:, 5));
hold on;
plot(posMatrix(:, 1), ones(noOfSamples) * j4limitUpper);
plot(posMatrix(:, 1), ones(noOfSamples) * j4limitLower);

%Via points
plot(0, viaMatrix(4, 1), 'ro');
plot(timeDivision, viaMatrix(4, 2), 'ro');
plot(timeDivision*2, viaMatrix(4, 3), 'ro');
plot(timeDivision*3, viaMatrix(4, 4), 'ro');
plot(timeDivision*4, viaMatrix(4, 5), 'ro');
hold off;
title("Position of joint 4");

figure(2)

x_data = posMatrix(:,6);
y_data = posMatrix(:,7);
plot(x_data, y_data, '-o');
title('X vs Y');
xlim([-400 400]);
ylim([-400 400]);
xlabel('X (mm)');
ylabel('Y (mm)');

hold off;
title("x and y overhead position");

%Velocity plots
figure(3);

subplot(1, 4, 1);
plot(velMatrix(:, 1), velMatrix(:, 2));
hold on;
plot(velMatrix(:, 1), ones(noOfSamples) * vMaxR);
plot(velMatrix(:, 1), ones(noOfSamples) * vMinR);
hold off;
title("Velocity of joint 1");

subplot(1, 4, 2);
plot(velMatrix(:, 1), velMatrix(:, 3));
hold on;
plot(velMatrix(:, 1), ones(noOfSamples) * vMaxR);
plot(velMatrix(:, 1), ones(noOfSamples) * vMinR);
hold off;
title("Velocity of joint 2");

subplot(1, 4, 3);
plot(velMatrix(:, 1), velMatrix(:, 4));
hold on;
plot(velMatrix(:, 1), ones(noOfSamples) * vMaxP);
plot(velMatrix(:, 1), ones(noOfSamples) * vMinP);
hold off;
title("Velocity of joint 3");

subplot(1, 4, 4);
plot(velMatrix(:, 1), velMatrix(:, 5));
hold on;
plot(velMatrix(:, 1), ones(noOfSamples) * vMaxR);
plot(velMatrix(:, 1), ones(noOfSamples) * vMinR);
hold off;
title("Velocity of joint 4");





%acceleration plots
figure(4);

subplot(1, 4, 1);
plot(accMatrix(:, 1), accMatrix(:, 2));
hold on;
plot(accMatrix(:, 1), ones(noOfSamples) * aMaxR);
plot(accMatrix(:, 1), ones(noOfSamples) * aMinR);
hold off;
title("Accel of joint 1");

subplot(1, 4, 2);
plot(accMatrix(:, 1), accMatrix(:, 3));
hold on;
plot(accMatrix(:, 1), ones(noOfSamples) * aMaxR);
plot(accMatrix(:, 1), ones(noOfSamples) * aMinR);
hold off;
title("Accel of joint 2");

subplot(1, 4, 3);
plot(accMatrix(:, 1), accMatrix(:, 4));
hold on;
plot(accMatrix(:, 1), ones(noOfSamples) * aMaxP);
plot(accMatrix(:, 1), ones(noOfSamples) * aMinP);
hold off;
title("Accel of joint 3");

subplot(1, 4, 4);
plot(accMatrix(:, 1), accMatrix(:, 5));
hold on;
plot(accMatrix(:, 1), ones(noOfSamples) * aMaxR);
plot(accMatrix(:, 1), ones(noOfSamples) * aMinR);
hold off;
title("Accel of joint 4");




