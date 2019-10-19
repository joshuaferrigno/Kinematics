% Joshua Ferrigno
% Department of Mechanical and Aerospace Engineering - UT Arlington

clear all;close all;clc;

% givens
LA = 4;
LC = 5;
w = 6;
g = -90*pi/180;

figure()                
hold on;                
grid on;                
title('Inverted Crank-slider [OPEN CONFIG]')

% inverted crank-slider [OPEN CONFIG]
for t1 = (60*pi/180) : (2*pi/180) : 300*pi/180
%     solve
    P = (LA*sin(t1)*sin(g)) + ((LA*cos(t1)-w)*cos(g));
    Q = -(LA*sin(t1)*cos(g)) + ((LA*cos(t1)-w)*sin(g));
    R = -LC*sin(g);
    
    S = R-Q;
    T = 2*P;
    U = R+Q;
    
    t3=2*atan2((-T-sqrt(T^2-(4*S*U))),2*S);
    t2=t3+g;
    q=((LA*sin(t1))-(LC*sin(t3)))/sin(t2);
    
 %     position
    PN = [0 ;0 ;0];         % N> frame
    PNA = [LA ;0 ;0];      % A> frame
    PAB = [-q ;0 ;0];        % B> frame
    PBC = [-LC ;0 ;0];       % C> frame
    
    
%     rotations
    RAN = rotz(t1*180/pi);
    RBN = rotz(t2*180/pi);
    RCN = rotz(t3*180/pi);
    
%     N> frame
    PNA_N = RAN*PNA;       % N> frame
    PAB_N = RBN*PAB;       % N> frame
    PBC_N = RCN*PBC;       % N> frame
    
    PNB_N = PNA_N+PAB_N;
    PNC_N = PNB_N+PBC_N;
    
%     graphing
    points = [PN PNA_N PNB_N PNC_N];
    
    plot(points(1,1:2),points(2,1:2),'-k','linewidth',4);
    hold on
    grid on
    plot(points(1,2:3),points(2,2:3),'-r','linewidth',4);
    plot(points(1,3:4),points(2,3:4),'-b','linewidth',4);
    axis([-3 7 -4 7])
    drawnow
    hold off
end


