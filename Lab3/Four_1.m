TI = 0.0552;
a = 1.55;
b= 10;
G = tf([TI 1], [TI 0])*tf([a], [1 b]);

rlocus(G);

K = 32.6;
VMAX_UPM = 11.75;
theta = 0;
%%
plot(Scope1(:,1), Scope1(:,2))
hold on
plot(Scope1(:,1), Scope1(:,3))

%%
plot(Scope2(:,1), Scope2(:,2))
%%