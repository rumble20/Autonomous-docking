classdef PIDHeadingController < handle
    % PIDHeadingController - PID heading controller (fallback for MPC failures)
    %
    % Based on autobargesim controlClass.LowLevelPIDCtrl.
    % Provides a simple PD/PID heading controller that can serve as fallback
    % when the NMPC solver fails.
    %
    % Usage:
    %   pid = PIDHeadingController(struct('K_p', 400, 'n_default', 70));
    %   [ctrl, pid] = pid.compute(psi_d, r, psi, dt);
    %
    % Author: Riccardo Legnini (ported from autobargesim)
    % Date: 2025
    
    properties
        K_p = 400           % Proportional gain
        T_i = 10            % Integral time constant
        T_d = 50            % Derivative time constant
        psi_d_old = 0       % Previous desired heading
        error_old = 0       % Previous heading error
        n_default = 70      % Default RPM when used as fallback
        delta_max_deg = 35  % Max rudder angle [deg]
    end
    
    methods
        function obj = PIDHeadingController(varargin)
            if nargin >= 1 && isstruct(varargin{1})
                cfg = varargin{1};
                if isfield(cfg, 'K_p'),     obj.K_p     = cfg.K_p;     end
                if isfield(cfg, 'T_i'),     obj.T_i     = cfg.T_i;     end
                if isfield(cfg, 'T_d'),     obj.T_d     = cfg.T_d;     end
                if isfield(cfg, 'n_default'), obj.n_default = cfg.n_default; end
                if isfield(cfg, 'delta_max_deg'), obj.delta_max_deg = cfg.delta_max_deg; end
            end
        end
        
        function [ctrl_out, obj] = compute(obj, psi_d, r, psi, h)
            % Compute PID control output
            %
            %   psi_d:  desired heading [rad]
            %   r:      yaw rate [rad/s]
            %   psi:    current heading [rad]
            %   h:      time step [s]
            %
            % Returns:
            %   ctrl_out: [delta_c (rad); n_c (RPM)]
            
            % Heading error (wrapped)
            error_psi = NavUtils.wrap_angle_diff_to_pmpi(psi, psi_d);
            
            % Desired yaw rate (derivative of reference)
            r_d = NavUtils.wrap_angle_diff_to_pmpi(psi_d, obj.psi_d_old) / h;
            
            % Integral term (trapezoidal)
            sum_error = error_psi + obj.error_old;
            
            % PID law
            delta_c = -obj.K_p * (error_psi + obj.T_d*(r - r_d) + (1/obj.T_i)*sum_error);
            
            % Saturate rudder command
            delta_max_rad = deg2rad(obj.delta_max_deg);
            delta_c = NavUtils.sat(delta_c, -delta_max_rad, delta_max_rad);
            
            % Update state
            obj.psi_d_old = psi_d;
            obj.error_old = error_psi;
            
            ctrl_out = [delta_c; obj.n_default];
        end
        
        function reset(obj, psi0)
            % Reset controller state
            if nargin < 2, psi0 = 0; end
            obj.psi_d_old = psi0;
            obj.error_old = 0;
        end
    end
end
