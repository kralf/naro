% SemesterProject Nanins

% simulate model

simOut = sim('simulation_lqr', 'StartTime', '0', 'StopTime', '5' , 'SaveState','on','StateSaveName','xoutNew');
simOutVars = simOut.who;
xoutNew = simOut.get('xoutNew');
sim_u = simOut.get('sim_u');