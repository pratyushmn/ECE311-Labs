close all
clc

plot(time, velocity, 'r', 'LineWidth', 1.5)
hold on
plot(simVelocity, 'b', 'LineWidth', 1.5)

title('Movement of Cart on Rails','FontWeight','bold','FontSize',15);
xlabel('Time [s]','FontWeight','bold','FontSize',14)
ylabel('Velocity [m/s]','FontWeight','bold','FontSize',14)
legend('Actual Velocity','Simulated Velocity');
save('Lab2_4_2'); 