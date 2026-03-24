

classdef RRTXData < handle
    properties
        % Tree structure
        nodes
        parents
        num_nodes

        % Cost arrays
        G
        LMC

        % Neighbor lists
        N_plus_r
        N_plus_r_count
        N_plus_0
        N_plus_0_count
        N_minus_r
        N_minus_r_count
        N_minus_0
        N_minus_0_count

        % Edge blocking flags (parallel to each neighbor list)
        is_edge_blocked_plus_0
        is_edge_blocked_plus_r
        is_edge_blocked_minus_0
        is_edge_blocked_minus_r

        % Children
        C_minus_T
        C_minus_T_count

        % Priority queue
        in_queue
        keys_1
        keys_2
        queue_list
        queue_count

        % Orphan status
        is_orphaned
    end

    methods
        function obj = RRTXData(k_max, max_neighbors)
            obj.nodes = zeros(k_max, 3);
            obj.parents = zeros(k_max, 1);
            obj.num_nodes = 0;

            obj.G = inf(k_max, 1);
            obj.LMC = inf(k_max, 1);

            obj.N_plus_r = zeros(k_max, max_neighbors, 'uint32');
            obj.N_plus_r_count = zeros(k_max, 1, 'uint16');
            obj.N_plus_0 = zeros(k_max, max_neighbors, 'uint32');
            obj.N_plus_0_count = zeros(k_max, 1, 'uint16');
            obj.N_minus_r = zeros(k_max, max_neighbors, 'uint32');
            obj.N_minus_r_count = zeros(k_max, 1, 'uint16');
            obj.N_minus_0 = zeros(k_max, max_neighbors, 'uint32');
            obj.N_minus_0_count = zeros(k_max, 1, 'uint16');

            obj.is_edge_blocked_plus_0  = false(k_max, max_neighbors);
            obj.is_edge_blocked_plus_r  = false(k_max, max_neighbors);
            obj.is_edge_blocked_minus_0 = false(k_max, max_neighbors);
            obj.is_edge_blocked_minus_r = false(k_max, max_neighbors);

            obj.C_minus_T = zeros(k_max, max_neighbors, 'uint32');
            obj.C_minus_T_count = zeros(k_max, 1, 'uint16');

            obj.in_queue = false(k_max, 1);
            obj.keys_1 = inf(k_max, 1);
            obj.keys_2 = inf(k_max, 1);
            obj.queue_list = zeros(k_max, 1, 'uint32');
            obj.queue_count = 0;

            obj.is_orphaned = false(k_max, 1);
        end

        function reset(obj)
            obj.nodes(:) = 0;
            obj.parents(:) = 0;
            obj.num_nodes = 0;

            obj.G(:) = inf;
            obj.LMC(:) = inf;

            obj.N_plus_r(:) = 0;
            obj.N_plus_r_count(:) = 0;
            obj.N_plus_0(:) = 0;
            obj.N_plus_0_count(:) = 0;
            obj.N_minus_r(:) = 0;
            obj.N_minus_r_count(:) = 0;
            obj.N_minus_0(:) = 0;
            obj.N_minus_0_count(:) = 0;

            obj.is_edge_blocked_plus_0(:) = false;
            obj.is_edge_blocked_plus_r(:) = false;
            obj.is_edge_blocked_minus_0(:) = false;
            obj.is_edge_blocked_minus_r(:) = false;

            obj.C_minus_T(:) = 0;
            obj.C_minus_T_count(:) = 0;

            obj.in_queue(:) = false;
            obj.keys_1(:) = inf;
            obj.keys_2(:) = inf;
            obj.queue_list(:) = 0;
            obj.queue_count = 0;

            obj.is_orphaned(:) = false;
        end
    end
end