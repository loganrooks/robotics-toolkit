function [x, y, theta, t] = pose(startPose, goalPose, optionalArgs)
    % Start and goal pose are of form [x, y, theta]
    % Optional arguments have form {Kp, Ka, Kb, h, length, maxIter}
    
    numOptionalArgs = length(optionalArgs);
    if numOptionalArgs > 6
        error('pose:TooManyInputs', ...
        'requires at most 6 optional inputs');
    end
    
    defaultOptionalArgs = {0.06, 0.4, -0.4, 0.01, 0.1, 20000};
    defaultOptionalArgs(1:numOptionalArgs) = optionalArgs;
    [Kp, Ka, Kb, h, length, maxIter] = defaultOptionalArgs{:};

    x = [startPose(1)];
    y = [startPose(2)];
    theta = [wrapToPi(startPose(3))];
    t = [0];
    
    poseError = [x(1) - goalPose(1), y(1) - goalPose(2), ...
        theta(1) - goalPose(3)];
    
    k = 2;
    
    while (k < maxIter && (abs(poseError(1)) > 0.01 || abs(poseError(2)) > 0.01 ...
        || abs(wrapToPi(poseError(3))) > 0.01))
    
        rho = sqrt(poseError(2)^2 + poseError(1)^2);
        alpha = wrapToPi(atan(poseError(2)/poseError(1)) - theta(k-1));
        beta = wrapToPi(goalPose(3) - theta(k-1) - alpha);
        
        v = Kp*rho;
        gamma = Ka*alpha + Kb*beta;
        omega = (v/length)*tan(gamma);
        
        x(k) = x(k-1) + h*v*cos(theta(k-1));
        y(k) = y(k-1) + h*v*sin(theta(k-1));
        theta(k) = wrapToPi(theta(k-1) + h*omega);
        t(k) = h*k;
        
        poseError = [x(k) - goalPose(1), y(k) - goalPose(2), ...
            theta(k) - goalPose(3)];
        
        k = k+1;
    end
end