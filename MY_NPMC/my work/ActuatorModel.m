classdef ActuatorModel < handle
    % ActuatorModel - Modular actuator model for rudder and propeller
    %
    % Extracted from container.m dynamics, inspired by autobargesim actuatorClass.
    % Provides rate-limited actuator responses for the plant simulation loop.
    %
    % Usage:
    %   act = ActuatorModel();
    %   [ctrl, act] = act.applyActuatorResponse([delta_c; n_c], dt);
    %
    % Author: Riccardo Legnini (refactored from autobargesim)
    % Date: 2025
    
    properties
        delta_max  = 35      % Max rudder angle [deg]
        Ddelta_max = 10      % Max rudder rate [deg/s]
        n_max      = 160     % Max shaft speed [RPM]
        n_min      = 0       % Min shaft speed [RPM]
        
        % Current actual values (for external rate limiting in plant sim)
        delta_actual = 0     % Actual rudder angle [rad]
        n_actual     = 0     % Actual shaft speed [RPM]
    end
    
    methods
        function obj = ActuatorModel(varargin)
            % Construct with optional config struct
            if nargin >= 1 && isstruct(varargin{1})
                cfg = varargin{1};
                if isfield(cfg, 'delta_max'),  obj.delta_max  = cfg.delta_max;  end
                if isfield(cfg, 'Ddelta_max'), obj.Ddelta_max = cfg.Ddelta_max; end
                if isfield(cfg, 'n_max'),      obj.n_max      = cfg.n_max;      end
                if isfield(cfg, 'n_min'),       obj.n_min      = cfg.n_min;      end
            end
        end
        
        function delta_dot = rudderDynamics(obj, delta_c, delta)
            % Compute rudder rate (matches container.m saturation + rate limiting)
            delta_max_rad  = deg2rad(obj.delta_max);
            Ddelta_max_rad = deg2rad(obj.Ddelta_max);
            
            delta_c_sat = NavUtils.sat(delta_c, -delta_max_rad, delta_max_rad);
            delta_dot   = delta_c_sat - delta;
            delta_dot   = NavUtils.sat(delta_dot, -Ddelta_max_rad, Ddelta_max_rad);
        end
        
        function n_dot = shaftDynamics(obj, n_c, n)
            % Compute shaft rate [RPM/s] (matches container.m Tm model)
            % n_c: commanded RPM, n: actual RPM
            
            n_revs   = n / 60;                          % rev/s
            n_c_revs = n_c / 60;                        % rev/s
            n_max_revs = obj.n_max / 60;
            
            n_c_clamped  = NavUtils.sat(n_c_revs, 0, n_max_revs);
            n_revs_safe  = max(n_revs, 0.01);
            
            if n_revs_safe > 0.3
                Tm = 5.65 / n_revs_safe;
            else
                Tm = 18.83;
            end
            
            n_dot = (1/Tm) * (n_c_clamped - n_revs_safe) * 60;  % RPM/s
        end
        
        function [ctrl_actual, obj] = applyActuatorResponse(obj, ctrl_command, dt)
            % Apply rate limiting and saturation to control commands
            %   ctrl_command: [delta_c (rad); n_c (RPM)]
            %   Returns: ctrl_actual = [delta_actual (rad); n_actual (RPM)]
            
            delta_c = ctrl_command(1);
            n_c     = ctrl_command(2);
            
            % Rudder: saturate + rate limit
            delta_max_rad  = deg2rad(obj.delta_max);
            Ddelta_max_rad = deg2rad(obj.Ddelta_max);
            delta_c_sat = NavUtils.sat(delta_c, -delta_max_rad, delta_max_rad);
            delta_diff  = NavUtils.sat(delta_c_sat - obj.delta_actual, ...
                                       -Ddelta_max_rad * dt, Ddelta_max_rad * dt);
            obj.delta_actual = obj.delta_actual + delta_diff;
            
            % Shaft: rate limit  (approx 50 RPM/s max rate)
            n_rate = 50;
            n_c_sat = NavUtils.sat(n_c, obj.n_min, obj.n_max);
            n_diff  = NavUtils.sat(n_c_sat - obj.n_actual, -n_rate*dt, n_rate*dt);
            obj.n_actual = obj.n_actual + n_diff;
            
            ctrl_actual = [obj.delta_actual; obj.n_actual];
        end
        
        function reset(obj, delta0, n0)
            % Reset actuator state
            if nargin < 2, delta0 = 0; end
            if nargin < 3, n0 = 0; end
            obj.delta_actual = delta0;
            obj.n_actual     = n0;
        end
    end
end
