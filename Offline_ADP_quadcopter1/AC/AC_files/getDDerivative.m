function xd = getDDerivative(x)
constants

    xd = ones(size(x));
    xd(:, 1) = (x(:, 2) - x(:, 1))/timeStep;
    for k = 2:(size(x, 2) - 1)
        xd(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*timeStep);
    end
    xd(:, end) = (x(:, end) - x(:, end - 1))/timeStep;
end