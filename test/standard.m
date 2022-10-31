[s] = textread('standard.txt');
x=s(:,1);
y=s(:,2);
z=s(:,3);
xx=0.1:0.05:0.95;
plot(xx,x,'ro-');hold on;plot(xx,y,'g*-');plot(xx,z,'b+-');hold off;
xlabel('coefficient for acceleration function');
ylabel('standard deviation');

