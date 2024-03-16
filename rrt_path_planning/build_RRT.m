
% This function will find the nearest node to the randomly sampled state
% and check if the robot will collide with any obstacles. If there are
% no collisions, the new node will be added to the tree and updated in RRT.

function [neighbors,tree] = build_RRT(rand_state,tree,obstacles,epsilon,neighbors)
    
    neighbor = nearest_neighbor(rand_state,tree);
    
    % nested function to find nearest neighbor to random state
    function neighbor = nearest_neighbor(rand_state,tree)

        x = rand_state(1)-tree(:,1);      % x distance per node
        y = rand_state(2)-tree(:,2);      % y distance per node
      
        [~,ind] = min(sqrt(x.^2+y.^2));   % find index of nearest neighbor
        neighbor = tree(ind,:);           % pull neighbor from tree

    end
    
    % calculate most direct path/vector and distance to random state
    vector = rand_state-neighbor;
    distance = norm(vector);        
    inc = epsilon*(vector/distance); % scale vector length by epsilon
    
    if distance <= epsilon           % if random state is reachable in one step
        
        new_node = rand_state;       % move to random state
        tree = [tree; new_node];     % add new node to tree
        
        neighbors = [neighbors; neighbor];  % add corresponding new neighbor to tree
    else
        
        new_node = neighbor + inc;   % move along vector inbetween points
        
        % collision checks
        pt_collide = collision_check_point(new_node(1),new_node(2),obstacles); 
        seg_collide = collision_check_segment(neighbor(1),neighbor(2),new_node(1),new_node(2),obstacles); 
        
        status = pt_collide + seg_collide;
    
        if status == 0                          % if no collisions
        
            tree = [tree; new_node];            % add new node to tree
            neighbors = [neighbors; neighbor];  % add new neighbor to tree
        
        end
    end                 % if there is a collision, no point/neighbor added
end