clear all
clc
close all
filename = 'data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

% for i=1:n
%     figure(i)
%     yy = i;
%     plot(Arr(:,yy),'r');
%     ylabel(cell2mat(VariableNames(yy)))
% end

plot(Arr(:,2),Arr(:,3),'k:','Linewidth',2); hold on;
plot(Arr(:,4),Arr(:,5),'r-','Linewidth',2);
legend('ref','act');
xlabel('x');
ylabel('z');
