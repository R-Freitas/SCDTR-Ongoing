
clear all
close all
clc

legends = {};
for occupancy1 = 0:1
    if occupancy1 == 0
        state1 = 'L';
        L1 = 90;
    elseif occupancy1 == 1
        state1 = 'H';
        L1 = 170;
    end
    for occupancy2 = 0:1
        if occupancy2 == 0
            state2 = 'L';
            L2 = 90;            
        elseif occupancy2 == 1
            state2 = 'H';
            L2 = 170;
        end
        for rho = 0.05:0.05:0.5    
            run('consensus_helper.m')
            legends{end+1,1} = sprintf('d_1 (rho = %f)',rho);
        end
        legend(legends);
        title(sprintf('Evolution of primal variables (Occupation levels: %s%s)', state1, state2))
        saveas(gcf,sprintf('%s%s.jpeg',state1,state2))
        close
    end
end