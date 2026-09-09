function plotter(simIter)

    %% PLOTTING
    o = [1 3 5 2 4 6];

    simO = simIter{1};
    time = simO.time;
    
    % Plot alpha results
    plotNames = ["X [m]", "Y [m]", "Z [m]", "U [m]", "V [m]", "W [m]"];
    figure;
    for i = 1:6
        subplot(3, 2, o(i)); hold on; grid on;
        plot(time, simO.alphaRef(i, :), 'g')
    
        for simt = simIter  % For every simulation
          sim = simt{1};
          plot(time, sim.states(i, :),  'b:')
        end
        
        xlabel('Time (s)'); ylabel(plotNames(i))
    end
    sgtitle("Alpha"); legend("Reference Path");
    
    % Plot beta results
    plotNames = ["PHI [rad]", "THE [rad]", "PSI [rad]", "P [rad/s]", "Q [rad/s]", "R [rad/s]"];
    figure;
    for i = 1:6
        subplot(3, 2, o(i)); hold on; grid on;
        plot(time, simO.betaRef(i, :), 'g')
    
        for simt = simIter  % For every simulation
          sim = simt{1};
          plot(time, sim.states(6 + i, :),  'b:')
        end
        
        xlabel('Time (s)'); ylabel(plotNames(i))
    end
    subplot(3, 2, 1); sgtitle("Beta"); legend("Reference Path");
    
    % Plot control signals
    plotNames = ["FT [N]", "TAUX [N*m]", "TAUY [N*m]", "TAUZ [N*m]"];
    figure;
    for i = 1:4
        subplot(2, 2, i); hold on; grid on;
    
        for simt = simIter  % For every simulation
          sim = simt{1};
          plot(time, sim.controls(i, :),  'b-')
        end
        
        xlabel('Time (s)'); ylabel(plotNames(i))
    end
    subplot(2, 2, 1); sgtitle("Controls")
    
    % Plot rotor signals (Singular)
    figure; hold on; grid on;
    for i = 1:4
        plot(time, simO.rotors(i, :))
    end
    xlabel('Time (s)'); ylabel("Omega [rad/s]"); 
    legend("O1", "O2", "O3", "O4"); sgtitle("Rotor Control Signals"); 
    
    % Plot 3D Path  (Singular)
    figure; hold on; grid on;
    plot3(simO.x(1),   simO.y(1),   simO.z(1),   'r*')
    plot3(simO.x(end), simO.y(end), simO.z(end), 'k*')
    plot3(simO.x,      simO.y,      simO.z)
    plot3(simO.F(1, :), simO.F(2, :), simO.F(3, :), 'g--')
    plot3(simO.f(1, :), simO.f(2, :), simO.f(3, :), 'r--')
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    sgtitle("Quadrotor Simulation")
    legend("Start", "End", "Simulated Trajectory", "Mod Trajectory", "Orig Trajectory")
    
    % Plot 3D Path 
    figure; hold on; grid on;
    
    plot3(simO.f(1, :), simO.f(2, :), simO.f(3, :), 'r--')
    
    for simt = simIter  % For every simulation
      sim = simt{1};
      plot3(sim.x(1),   sim.y(1),   sim.z(1),   'r*')
      plot3(sim.x(end), sim.y(end), sim.z(end), 'k*')
      plot3(sim.x,      sim.y,      sim.z)
    end
    
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    sgtitle("Quadrotor Simulation")
    legend("Orig Trajectory")
end