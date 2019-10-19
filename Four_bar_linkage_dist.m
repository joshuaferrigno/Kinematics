% Joshua Ferrigno
% Department of Mechanical and Aerospace Engineering - UT Arlington

clear all;close all;clc;

v = VideoWriter('4bar.avi');
open(v);

% values
LA = 1;
LB = 3;
LC = 1.5;
L1 = 2;
L2 = .5;
w  = 3;
h = 1;
t1=pi/6;

figure()          
hold on
title('Four Bar Linkage')

% 4 bar
for i=0 : 1 : 720
    t1=i*pi/180;

    % alphas
    a1=LA^2-LB^2+LC^2+w^2+h^2-2*LA*(w*cos(t1)+h*sin(t1));
    a2=2*LA*LC*cos(t1)-2*LC*w;
    a3=2*LA*LC*sin(t1)-2*LC*h;

    a4=LA^2+LB^2-LC^2+w^2+h^2-2*LA*(w*cos(t1)+h*sin(t1));
    a5=2*LA*LB*cos(t1)-2*LB*w;
    a6=2*LA*LB*sin(t1)-2*LB*h;

    % quadratic components
    A=a1-a2;
    B=2*a3;
    C=a1+a2;

    D=a4-a5;
    E=2*a6;
    F=a4+a5;

    % angles
    t3=2*atan2((-B-(B^2-(4*A*C))^.5),(2*A));
    t2=2*atan2((-E+(E^2-(4*D*F))^.5),(2*D));

    % position and rotation
    PN  = [0;0;0];              % position of point N
    PNA = [LA;0;0];             % A> frame
    PAB = [LB;0;0];             % B> frame
    PBC = [LC;0;0];             % C> frame
    PAT1 = [L1;L2;0];

    RAN = rotz(t1*180/pi);
    RBN = rotz(t2*180/pi);
    RCN = rotz(t3*180/pi);

    % N> frame
    PNA_N = RAN*PNA;              
    PAB_N = RBN*PAB;          
    PBC_N = RCN*PBC;
    PAT_N = RBN*PAT1;

    PNB_N = PNA_N+PAB_N;
    PNC_N = PNB_N+PBC_N;
    PNT_N = PNA_N+PAT_N;

    % graphing
    points = [PN PNA_N PNB_N PNC_N];
    if rem(i+1,3) == 0
        Tpoints(i+1,:) = PNT_N;
    else
        Tpoints(i+1,:) = [100 100 0];
    end

    L0 = plot(Tpoints(:,1),Tpoints(:,2),'.','Color',[0.75, 0, 0.75],'MarkerSize',10);
    hold on;  
   
    L11 = plot(PNA_N(1),PNA_N(2),'r.','MarkerSize',45);
    plot(PNB_N(1),PNB_N(2),'r.','MarkerSize',45)
    L22 =plot(PN(1),PN(2),'b.','Color',[0, 0.4470, 0.7410],'MarkerSize',45);
    plot(PNC_N(1),PNC_N(2),'b.','Color',[0, 0.4470, 0.7410],'MarkerSize',45)
    plot(points(1,:),points(2,:),'-k','linewidth',5);
    axis([-1.25 3.25 -1.25 3.25])
    grid on;   
    title('Four Bar Linkage')
    legend('location','northwest')
    legend([L0 L11 L22],{'Tool','Pins','Ground'},'FontSize',12)

    drawnow
    frame = getframe(gcf);
    writeVideo(v,frame);
    hold off
end

close(v)