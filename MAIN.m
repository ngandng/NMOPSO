pclc;
clear;
close all;
initTime = 0;
save('InitTimeNMOPSO6.mat','initTime');
for iter = 1:50
    disp(['The number of turn ' num2str(iter)]);
    NMOPSO;
end