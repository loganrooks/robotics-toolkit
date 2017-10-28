function [x, y, theta, t] = pursuit(startPose, path, Kh, Kv, Ki)
    length = 0.1;
    distance = 0.01;
    h = 0.01;
    
    x = [startPose(1)];
    y = [startPose(2)];
    theta = [wrapToPi(startPose(3))];
    t = [0];
    integral = 0;
    
    k = 1;
    
    xPath = k*h;
    yPath = path(xPath);
    relativePose = [xPath - x(k), yPath - y(k), atan((yPath - y(k))/(xPath - x(k)))];
    
    for k = 2:2*pi/h
        xPath = k*h;
        yPath = path(xPath);
        
        error = sqrt(relativePose(1)^2 + relativePose(2)^2) - distance;
        integral = integral + error;
        
        v = Kv*error + Ki*integral;
        v = min(max(v,0), 2);
        gamma = Kh*wrapToPi(relativePose(3) - theta(k-1));
        gamma = min(max(gamma,-pi/2), pi/2);
        
        omega = (v/length)*tan(gamma);
        
        t(k) = xPath;
        x(k) = x(k-1) + h*v*cos(theta(k-1));
        y(k) = y(k-1) +h*v*sin(theta(k-1));
        theta(k) = wrapToPi(theta(k-1) + h*omega);
        
        relativePose = [xPath - x(k), yPath - y(k), atan((yPath - y(k))/(xPath - x(k)))];   
        
    end
        