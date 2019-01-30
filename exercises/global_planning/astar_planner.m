% This file implements a basic A* path planner
%
% The function returns a planner 'object' that is initialized with the
% given obstacle map, start and goal position, but no path is planned at
% this stage.
% The actual planning is done in the 'member function' run(), which returns
% the waypoint sequence and optionally some debug information.
%
function obj = astar_planner(map, start, goal)
    % some abbreviations
    sz = size(map);
    w = sz(2);
	h = sz(1);
	
    % A* per-cell data
	closed = false(size(map));      % boolean matrix: true for each cell in closed list (initially, no cell is closed)
	g_cost = zeros(size(map));      % matrix of g_cost. Only cells in open/closed list contain meaningful values.
    f_cost = zeros(size(map));      % matrix of f_cost = g_cost + heuristic. This is initialized and updated for each cell in the open list
                                    % Note: the heuristic (h_cost) is not explicitely stored, since it can always be retrieved from f_cost - g_cost	
    parent = zeros(size(map), 'uint32');	% matrix of 'predecessor' cells (valid for open/closed cells only)
    % variables implementing the Open List as a Binary Heap
    openHeap = zeros([1, numel(map)], 'int32');% vector of indices of all cells in the open list
                                    % The vector is always initialized to maximum size...
	nOpenCells = uint32(0);                 % but only the first nOpenCells are occupied
    openHeapPos = zeros(size(map), 'uint32'); % matrix of positions of cells in openHeap. Only valid for open cells.
                                    % This 'back-reference' is required for updating the heap after after cost changes

    errorMessage = '';

    start = round(start); goal = round(goal);
    % start & goal as x/y coordinates and cell index
    x_start = start(1);
    y_start = start(2);
    startCell = sub2ind(sz, y_start, x_start);
    x_goal = goal(1);
    y_goal = goal(2);        
    goalCell = sub2ind(sz, y_goal, x_goal);
    
    if any(start) < 1 || start(1) > w || start(2) > h
        errorMessage = 'Start position is not in map';
    elseif any(goal) < 1 || goal(1) > w || goal(2) > h
        errorMessage = 'Goal position is not in map';
    elseif map(start(2), start(1))
        errorMessage = 'Start position blocked';
    elseif map(goal(2), goal(1))
        errorMessage = 'Goal position blocked';
    else
        % Initialize the open list (Heap) with the start cell
        g_cost(startCell) = 0;
        delta = abs([x_start - x_goal, y_start - y_goal]);
        f_cost(startCell) = 10 * abs(delta(1) - delta(2)) + 14 * min(delta);
        openHeap_add(startCell);
    end
                 
    % add 'member functions' to planner 'object'
    obj.run = @run; 
    obj.getCellInfo = @getCellInfo;
    
    % member function that does the actual planning
    % For better understanding, the number of iterations performed in each 
    % invocation can be limited.
    %
    % return values:
    %   waypoints: 2xN array the coordinates of the cells that constitute
    %              the path. One [x;y] pair per column. Empty if a path
    %              has not been found (yet)
    %   errMsg, cellTypes, heapHead: debug data, see comments in code below 
    %                                for code below for further information
    function [waypoints, errMsg, cellTypes, heapHead] = run(iterations)
        if nargin < 1; iterations = inf; end
        
        waypoints = [];     
        
        if isempty(errorMessage)
            
            % Prepare some structures for efficient neighbourhood processing
            % Order: 7 6 5
            %        8 X 4
            %        1 2 3
            neighbourCosts = [14 10 14 10 14 10 14 10]; % cost between cell and neighbour
            neighbourOffsets = [-h + 1, 1, h + 1, h, h - 1, -1, -h - 1, -h]; % vector offset from cell to neighbour. 
            % (In Matlab, matrices can also be treated as a linear vectors, where the matrix elements are stored column-wise)
            
            % Run A* iterations, while the solution has not been found and there are still open cells
            while ~closed(goalCell) && nOpenCells > 0 && iterations > 0 
                iterations = iterations - 1;
                
                % TODO: Remove open cell with least f_cost and put it into Closed List
                

                
                
                % Determine set of existing neighbours.
                [y, x] = ind2sub(sz, cell);        
                validNeighbours = true(1, 8);
                if (x == 1); validNeighbours([1 7 8]) = false; end
                if (x == w); validNeighbours([3 4 5]) = false; end
                if (y == 1); validNeighbours([1 2 3]) = false; end
                if (y == h); validNeighbours([5 6 7]) = false; end		

                for i = 1:8
                    if validNeighbours(i)
                        neighbourCell = cell + neighbourOffsets(i);
                        
                        % Ignore obstacles and closed cells
                        if map(neighbourCell) || closed(neighbourCell); continue; end

                        heapPos = openHeapPos(neighbourCell);
                        if heapPos > 0
                            % If a cell has a valid heapPos (> 0), it is in
                            % the Open List (Heap)
                            
                            % TODO: check if its g_cost can be reduced
                            % (Do not forget to update the Heap!)

                        
                        
                        
                        
                            
                            
                        else
                            % cell is unknown 
                            % TODO: initialize cell and add it to the Open Heap
                            
                            
                            
                            
                            
                        end
                    end
                end
            end % end of A* loop
            
            if closed(goalCell) % we found a path                
                
                % TODO: extract and return the path
                

                
                
                waypoints = 
                
            elseif nOpenCells == 0
                % no path found and no open cells left -> there is no
                % unblocked path between start and goal
                errorMessage = 'No Path found!';
            end
        end

        % prepare debug data, if requested by the caller
        if nargout >= 2
            errMsg = errorMessage;
        end
        if nargout >= 3        
            % Map of cell types with values:
            % 0 = unknown (free) cell
            % 1 = blocked cell (obstacle)
            % 2 = cell in Open List
            % 3 = cell in Closed List
            % 4 = Open Cell with min. f_cost (head of Open Heap)
            cellTypes = uint8(map);
            cellTypes(openHeapPos > 0) = 2;
            cellTypes(closed) = 3;        
            if nOpenCells > 0
                cellTypes(openHeap(1)) = 4;
            end
        end
        if nargout >= 4
            % x/y coordinates of the Open Cell with min. f_cost
            if nOpenCells > 0
                [y, x] = ind2sub(sz, openHeap(1));
                heapHead = double([x; y]);
            else heapHead = zeros(2, 0);
            end
        end
    end    
    
    % member function for accessing detailed information for the given cell
    function cellInfo = getCellInfo(x, y)
        cell = sub2ind(sz, y, x);
        cellInfo.g_cost = g_cost(cell);
        cellInfo.f_cost = f_cost(cell);
        if map(cell)
            cellInfo.type = 'Obstacle';
        elseif closed(cell)
            cellInfo.type = 'Closed';
        elseif openHeapPos(cell) > 0
            cellInfo.type = 'Open';
        else cellInfo.type = 'Unknown';
        end
        parentCell = parent(cell);
        if parentCell > 0
            [yParent, xParent] = ind2sub(sz, parentCell);
            cellInfo.parent = [xParent; yParent];
        end       
    end

    % ======= Functions for managing the Open List as a Binary Heap =======
    % A heap is a semi-ordered tree structure, where no child has a smaller
    % value than its parent
    
    % Add a cell to the Heap
    function openHeap_add(cell)
        % append new cell at the end...
        nOpenCells = nOpenCells + 1;
        openHeap(nOpenCells) = cell;
        openHeapPos(cell) = nOpenCells;                		
        % ... and propagate it upwards
        openHeap_propagateToHead(cell);
    end
    % Propagate cell upwards until the heap property
    % is restored (used by openHeap_add and after the f_cost reduction)
    function openHeap_propagateToHead(cell)
        idx = openHeapPos(cell);
        while idx > 1
            idxParent = bitshift(idx, -1); % C: idx >> 1
            if f_cost(openHeap(idx)) <= f_cost(openHeap(idxParent))
                % swap with parent
                openHeapPos(openHeap(idx)) = idxParent;
                openHeapPos(openHeap(idxParent)) = idx;
                temp = openHeap(idx);
                openHeap(idx) = openHeap(idxParent);
                openHeap(idxParent) = temp;
                idx = idxParent;
            else break;
            end
        end        
    end
    % remove and return the head element (element with smallest f_cost)
    % from the heap
	function head = openHeap_popHead()
		head = openHeap(1);
		% Make the tail element the new head...
        openHeap(1) = openHeap(nOpenCells);
        openHeapPos(openHeap(1)) = 1;
        nOpenCells = nOpenCells - 1;        
        % ...and let it sink downwards until the heap property is restored 
        idx = 1;
        while true            
            idxPrev = idx;
            idxLeft = 2 * idx;
            idxRight = idxLeft + 1;                        
            if idxRight <= nOpenCells
                if f_cost(openHeap(idx)) > f_cost(openHeap(idxLeft)); idx = idxLeft; end
                if f_cost(openHeap(idx)) > f_cost(openHeap(idxRight)); idx = idxRight; end                    
            elseif idxLeft <= nOpenCells
                if f_cost(openHeap(idx)) > f_cost(openHeap(idxLeft)); idx = idxLeft; end
            end
            
            if idx ~= idxPrev
                openHeapPos(openHeap(idx)) = idxPrev;
                openHeapPos(openHeap(idxPrev)) = idx;
                temp = openHeap(idx);
                openHeap(idx) = openHeap(idxPrev);
                openHeap(idxPrev) = temp;
            else break;
            end
        end        
    end
end
