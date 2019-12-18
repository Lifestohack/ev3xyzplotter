T = readtable('ymotor.txt');
y = T{:,:}
input = y(:,1)
measured = y(:,1)
out = y(:,2)

plot(input)
hold on

plot(out)