close all
figure
hold on

LW = 3; FS = 14; MS = 8;

plot(KKneeHist15,Vmax15,'b','LineWidth',LW);

plot(KKneeHist12,Vmax12,'y','LineWidth',LW);

plot(KKneeHist10,Vmax10,'r','LineWidth',LW);

plot(KKneeHist5,Vmax5,'g','LineWidth',LW);

[V15, i15] = max(Vmax15);
plot(KKneeHist(i15),V15,'ro','MarkerSize',MS)

load('/home/gbledt/MIT/RobotSoccerSimulation/Data/BallVelocityVsKKneev12.mat')
[V12, i12] = max(Vmax12);
plot(KKneeHist(i12),V12,'ro','MarkerSize',MS)

load('/home/gbledt/MIT/RobotSoccerSimulation/Data/BallVelocityVsKKneev10o.mat')
[V10, i10] = max(Vmax10);
plot(0.18,V10,'ro','MarkerSize',MS)

load('/home/gbledt/MIT/RobotSoccerSimulation/Data/BallVelocityVsKKneev5o.mat')
[V5, i5] = max(Vmax5);
plot(KKneeHist(i5),V5,'ro','MarkerSize',MS)

plot([KKneeHist(i5),0.18,KKneeHist(i12),KKneeHist(i15)],...
    [V5,V10,V12,V15],'--','LineWidth',LW-1)

set(gcf,'Color','w')
axis([0 1 0 5])
xlabel('Knee Stiffness, K [Nm/rad]','FontSize',FS);
ylabel('Maximum Ball Velocity, v [m/s]','FontSize',FS);
legend('Hip Velocity 1','Hip Velocity 2','Hip Velocity 3','Hip Velocity 4')
