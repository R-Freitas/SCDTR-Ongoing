
for rho = 0.1:0.05:1    
    run('consensus_helper.m')
    saveas(gcf,sprintf('rho_%f.jpeg',rho))
    close
end