clear all
clc
close all

system('./run_unix');

filename = 'data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

figure(1)
subplot(2,1,1);
plot(Arr(:,1),Arr(:,2),'r'); hold on;
plot(Arr(:,1),Arr(:,4),'b-.','Linewidth',2);
xlabel('time');
ylabel(cell2mat(VariableNames(2)))
legend('act','ref');
subplot(2,1,2);
plot(Arr(:,1),Arr(:,3),'r'); hold on;
plot(Arr(:,1),Arr(:,5),'b-.','Linewidth',2);
legend('act','ref');
xlabel('time');
ylabel(cell2mat(VariableNames(3)))



% for i=1:n
%     figure(i)
%     yy = i;
%     plot(Arr(:,yy),'r');
%     ylabel(cell2mat(VariableNames(yy)))
% end
