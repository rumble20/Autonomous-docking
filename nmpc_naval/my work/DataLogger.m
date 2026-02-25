classdef DataLogger < handle
    % DataLogger - Comprehensive data logging and visualization
    %
    % Logs all simulation data and generates thesis-ready plots
    %
    % Author: Riccardo Legnini
    % Date: 2025-02-23
    
    properties
        data struct
        save_enabled logical
    end
    
    methods
        function obj = DataLogger()
            obj.data.time = [];
            obj.data.state = struct('x', [], 'y', [], 'psi', [], ...
                                     'u', [], 'v', [], 'r', []);
            obj.data.control = struct('tau_surge', [], 'tau_sway', [], 'tau_yaw', []);
            obj.data.risk = struct('CRI', [], 'TCPA', [], 'DCPA', []);
            obj.data.thruster = struct('T1', [], 'T2', [], 'alpha1', [], 'alpha2', []);
            obj.save_enabled = true;
        end
        
        function record(obj, t, state, control_output, risk_assessment)
            % Record one timestep of data
            
            obj.data.time(end+1) = t;
            
            % State
            obj.data.state.x(end+1) = state.x;
            obj.data.state.y(end+1) = state.y;
            obj.data.state.psi(end+1) = state.psi;
            obj.data.state.u(end+1) = state.u;
            obj.data.state.v(end+1) = state.v;
            obj.data.state.r(end+1) = state.r;
            
            % Control
            if ~isempty(control_output)
                obj.data.control.tau_surge(end+1) = control_output.tau(1);
                obj.data.control.tau_sway(end+1) = control_output.tau(2);
                obj.data.control.tau_yaw(end+1) = control_output.tau(3);
            end
            
            % Risk
            obj.data.risk.CRI(end+1) = risk_assessment.CRI;
            
            if ~isempty(risk_assessment.obstacles)
                obj.data.risk.TCPA(end+1) = risk_assessment.obstacles(1).TCPA;
                obj.data.risk.DCPA(end+1) = risk_assessment.obstacles(1).DCPA;
            else
                obj.data.risk.TCPA(end+1) = inf;
                obj.data.risk.DCPA(end+1) = inf;
            end
        end
        
        function plot(obj)
            % Generate comprehensive plot suite
            
            figure('Name', 'Autonomous Docking Simulation Results', ...
                   'Position', [100, 100, 1400, 900]);
            
            % Plot 1: Trajectory (top view)
            subplot(3, 3, [1, 2, 4, 5]);
            plot(obj.data.state.x, obj.data.state.y, 'b-', 'LineWidth', 2);
            hold on; grid on; axis equal;
            
            % Mark start and end
            plot(obj.data.state.x(1), obj.data.state.y(1), 'go', ...
                'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(obj.data.state.x(end), obj.data.state.y(end), 'ro', ...
                'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
            
            % Plot heading vectors
            skip = max(1, floor(length(obj.data.time)/20));
            quiver(obj.data.state.x(1:skip:end), obj.data.state.y(1:skip:end), ...
                   cos(obj.data.state.psi(1:skip:end)), ...
                   sin(obj.data.state.psi(1:skip:end)), ...
                   10, 'k', 'LineWidth', 1);
            
            xlabel('East [m]'); ylabel('North [m]');
            title('Vessel Trajectory');
            legend('Location', 'best');
            
            % Plot 2: Velocities
            subplot(3, 3, 3);
            plot(obj.data.time, obj.data.state.u, 'b-', 'LineWidth', 1.5); hold on;
            plot(obj.data.time, obj.data.state.v, 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]'); ylabel('Velocity [m/s]');
            legend('u (surge)', 'v (sway)');
            title('Body-Frame Velocities');
            
            % Plot 3: Heading and yaw rate
            subplot(3, 3, 6);
            yyaxis left
            plot(obj.data.time, obj.data.state.psi * 180/pi, 'b-', 'LineWidth', 1.5);
            ylabel('Heading [°]');
            yyaxis right
            plot(obj.data.time, obj.data.state.r * 180/pi, 'r-', 'LineWidth', 1.5);
            ylabel('Yaw Rate [°/s]');
            xlabel('Time [s]');
            grid on;
            title('Heading and Yaw Rate');
            
            % Plot 4: Control forces
            subplot(3, 3, 7);
            plot(obj.data.time(1:length(obj.data.control.tau_surge)), ...
                 obj.data.control.tau_surge/1e3, 'b-', 'LineWidth', 1.5); hold on;
            plot(obj.data.time(1:length(obj.data.control.tau_sway)), ...
                 obj.data.control.tau_sway/1e3, 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]'); ylabel('Force [kN]');
            legend('\tau_{surge}', '\tau_{sway}');
            title('Control Forces');
            
            % Plot 5: Yaw moment
            subplot(3, 3, 8);
            plot(obj.data.time(1:length(obj.data.control.tau_yaw)), ...
                 obj.data.control.tau_yaw/1e6, 'k-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]'); ylabel('Moment [MNm]');
            title('Control Yaw Moment');
            
            % Plot 6: Collision Risk
            subplot(3, 3, 9);
            plot(obj.data.time, obj.data.risk.CRI, 'r-', 'LineWidth', 2);
            hold on;
            yline(0.7, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Risk Threshold');
            grid on;
            xlabel('Time [s]'); ylabel('CRI [-]');
            title('Collision Risk Index');
            ylim([0, 1]);
            legend('Location', 'best');
            
            % Save figure
            if obj.save_enabled
                saveas(gcf, 'docking_simulation_results.png');
                fprintf('✓ Plots saved to docking_simulation_results.png\n');
            end
        end
        
        function exportData(obj, filename)
            % Export data to MAT file
            data = obj.data;
            save(filename, 'data');
            fprintf('✓ Data exported to %s\n', filename);
        end
        
        function printSummary(obj)
            % Print simulation summary statistics
            
            fprintf('\n=== SIMULATION SUMMARY ===\n');
            fprintf('Duration: %.1f seconds\n', obj.data.time(end));
            fprintf('Distance traveled: %.1f meters\n', ...
                sum(sqrt(diff(obj.data.state.x).^2 + diff(obj.data.state.y).^2)));
            fprintf('Max speed: %.2f m/s\n', max(sqrt(obj.data.state.u.^2 + obj.data.state.v.^2)));
            fprintf('Max heading rate: %.2f °/s\n', max(abs(obj.data.state.r))*180/pi);
            fprintf('Max control surge force: %.1f kN\n', max(abs(obj.data.control.tau_surge))/1e3);
            fprintf('Max CRI: %.2f\n', max(obj.data.risk.CRI));
            fprintf('=========================\n\n');
        end
    end
end
