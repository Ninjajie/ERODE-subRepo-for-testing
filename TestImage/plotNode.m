x = dlmread('x.txt','\t');
y = dlmread('y.txt','\t');
id = dlmread('index.txt','\t');
bx = dlmread('branchx.txt');
by = dlmread('branchy.txt');
pri = dlmread('pri.txt');
c = zeros(length(id), 3);
for n = 1 : length(id)
    if id(n) == 1
        c(n, :) = [1, 0, 0];
    elseif id(n) == 2
        c(n,:) = [0, 1, 0];
    elseif id(n) == 3
        c(n,:) = [0, 0, 1];
    elseif id(n) == 4
        c(n,:) = [1, 1, 0];
%     elseif id(n) == 0
%         c(n,:) = [0, 1, 1];
    end
    
end
tick = zeros(15);
for i = 1 : 15
    tick(i) = 7.5 * (i - 1);
end
figure;
scatter(x,y,[],c,'filled');
hold on;
for i = 1:length(bx)
    plot([bx(i,1) bx(i,2)], [by(i,1), by(i,2)]);
end
grid on;
set(gca, 'xtick', [0:37.5:1030]);
set(gca, 'ytick',[0:37.5:1030]);
a = [1:length(x)]';
b = num2str(pri);
cc = cellstr(b);
text(x + 0.5, y + 0.5, cc);

