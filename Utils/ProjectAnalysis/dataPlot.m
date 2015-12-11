close all
figure
hold on

LW = 3; FS = 14; MS = 8;


plot(KKneeHist15,Vmax15,'b','LineWidth',LW);

%plot(KKneeHist12,Vmax12,'y','LineWidth',LW);

plot(KKneeHist10,Vmax10,'r','LineWidth',LW);

plot(KKneeHist5,Vmax5,'g','LineWidth',LW);

set(gcf,'Color','w')
axis([0 1 0 5])
xlabel('Knee Stiffness, K [Nm/rad]','FontSize',FS);
ylabel('Maximum Ball Velocity, v [m/s]','FontSize',FS);
legend('V = 15','V = 10','V = 5')

plot(Kexp5,Vexp5,'go',Kexp10,Vexp10,'r.',Kexp15,Vexp15,'b*')