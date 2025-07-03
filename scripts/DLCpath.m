% DLCReference parameters (sharper example)
shape = 2.4;
dx1 = 25;
dx2 = 21.95;
dy1 = 4.05;
dy2 = 5.7;
Xs1 = 27.19;
Xs2 = 56.45;
X_offset = 30;
Y_offset = 0.2;

% Longitudinal path (meters)
X = linspace(0, 200, 1000);

% Initialize reference arrays
YRef = zeros(size(X));
YawRef = zeros(size(X));

for i = 1:length(X)
    if X(i) >= X_offset
        z1 = shape/dx1 * (X(i) - X_offset - Xs1) - shape/2;
        z2 = shape/dx2 * (X(i) - X_offset - Xs2) - shape/2;
        
        YawRef(i) = atan( dy1*(1./cosh(z1)).^2*(1.2/dx1) - dy2*(1./cosh(z2)).^2*(1.2/dx2));
        YRef(i) = dy1/2*(1 + tanh(z1)) - dy2/2*(1 + tanh(z2)) + Y_offset;
    else
        YawRef(i) = 0;
        YRef(i) = Y_offset;
    end
end

% Plot lateral path
figure;
plot(X, YRef, 'b', 'LineWidth', 2);
grid on;
xlabel('Longitudinal Distance X (m)');
ylabel('Lateral Position Y (m)');
title('Desired Path from DLCReference');
legend('Y_{Ref}');
