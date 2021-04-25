%绘制图像前运行pre_plot.m & motor_valite.slx
figure(1);
plot(experiment_time,experiment,'LineWidth',2.5);hold on;
plot(yout{1}.Values,'r','LineWidth',2.5);hold on;
ylim([-300 300]);
ylabel('Rotating Speed(Rpm)');hold on;
xlabel('Time(s)');hold on;
title('Comparison of Simulated and Measured Curves','position',[1.9,310]);hold on;
set(gca,'fontsize',18);%设置坐标轴字体大小
set(gca,'linewidth',2.5); %坐标 线粗0.5磅
set(get(gca,'XLabel'),'FontSize',18);%图上文字为8 point或小5号
set(get(gca,'YLabel'),'FontSize',18);
set(get(gca,'TITLE'),'FontSize',18);
legend('Measured','Simulated','Best');

figure(2);
plot(yout{1}.Values,'r','LineWidth',2.5);hold on;
ylim([-300 300]);
title('Simulation Output');hold on;
ylabel('Rotating Speed(Rpm)');hold on;
xlabel('Time(s)');
set(gca,'fontsize',18);%设置坐标轴字体大小
set(gca,'linewidth',2.5); %坐标 线粗0.5磅
set(get(gca,'XLabel'),'FontSize',18);%图上文字为8 point或小5号
set(get(gca,'YLabel'),'FontSize',18);
set(get(gca,'TITLE'),'FontSize',18);

figure(3);
plot(input,'LineWidth',2.5);hold on;
ylim([-1 6]);
title('Step Signal');hold on;
ylabel('Voltage(V)');hold on;
xlabel('Time(s)');
set(gca,'fontsize',18);%设置坐标轴字体大小
set(gca,'linewidth',2.5); %坐标 线粗0.5磅
set(get(gca,'XLabel'),'FontSize',18);%图上文字为8 point或小5号
set(get(gca,'YLabel'),'FontSize',18);
set(get(gca,'TITLE'),'FontSize',18);
