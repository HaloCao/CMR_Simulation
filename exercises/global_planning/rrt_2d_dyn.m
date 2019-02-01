function rrt_2d_dyn()
    
    sizeX = [0 100];
    sizeY = [0 100];
    start = [50; 90];    
    
    maxExtensionDistance = 4;
    goal = [60; 10];
    goalBias = 0.1;
    goalDistance = 0.5;
    
    useFixedObstacles = true;
    nRandomObstacles = 10;
    
    f = figure('Renderer', 'OpenGL', 'NumberTitle', 'off', 'Name', 'Simple RRT in 2D');        
    ax = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'DataAspectRatio', [1 1 1], 'XLim', sizeX, 'YLim', sizeY, 'Layer', 'top');
    
    % draw goal
    rectangle('Parent', ax, 'Position', [goal(1) - goalDistance, goal(2) - goalDistance, 2 * goalDistance, 2 * goalDistance], 'Curvature', [1 1], 'FaceColor', [1 0.5 0.5], 'EdgeColor', 'none');
    patch('Parent', ax, 'XData', goal(1), 'YData', goal(2), 'EdgeColor', 'none', 'Marker', 'x', 'MarkerEdgeColor', [1 0 0], 'MarkerSize', 10, 'LineWidth', 2);
    
    % add obstacles
    obstacles = zeros(0, 4);
    if useFixedObstacles
        obstacles(end + 1, :) = [25, 20, 50, 20];
        %obstacles(end + 1, :) = [30, 20, 50, 20];
        %obstacles(end + 1, :) = [5, 20, 20, 50];    
    end
    nCreatedObstacles = 0;
    
    randObstacleRelSize = [0.01, 0.2];
    
    % random number generator for random obstacles
    rng(0);
    while nCreatedObstacles < nRandomObstacles
        wRange = randObstacleRelSize * (sizeX(2) - sizeX(1));
        w = rand() * (wRange(2) - wRange(1)) + wRange(1);
        hRange = randObstacleRelSize * (sizeY(2) - sizeY(1));
        h = rand() * (hRange(2) - hRange(1)) + hRange(1);
         
        left = sizeX(1) + rand() * ((sizeX(2) - sizeX(1)) - w);
        bot = sizeY(1) + rand() * ((sizeY(2) - sizeY(1)) - h);
        if start(1) <= left || start(1) >= left + w || start(2) <= bot || start(2) >= bot + h
            obstacles(end + 1, :) = [left, bot, w, h];
            nCreatedObstacles = nCreatedObstacles + 1;
        end
    end
    
    for i = 1:size(obstacles, 1)
        rectangle('Parent', ax, 'Position', obstacles(i, :), 'FaceColor', 0.5 * [1 1 1], 'EdgeColor', 'none');
    end
    
    pad = 4;
    x = pad;
    uicontrol('style', 'text', 'String', 'More Iterations:', 'Units', 'pixels', 'Position', [x, 8, 120, 16]); x = x + 120 + pad;
    hIterateBtns(1) = uicontrol('style', 'pushbutton', 'String', '10', 'Units', 'pixels', 'Position', [x, 4, 48, 24], 'Callback', @(varargin)iterate(10)); x = x + 48 + pad;
    hIterateBtns(2) = uicontrol('style', 'pushbutton', 'String', '100', 'Units', 'pixels', 'Position', [x, 4, 48, 24], 'Callback', @(varargin)iterate(100)); x = x + 48 + pad;
    hIterateBtns(3) = uicontrol('style', 'pushbutton', 'String', '1000', 'Units', 'pixels', 'Position', [x, 4, 48, 24], 'Callback', @(varargin)iterate(1000)); x = x + 48 + pad;    
    

    root.pos = start;
    root.parent = [];
    root.children = [];
    root.nodeHandle = patch('Parent', ax, 'XData', root.pos(1), 'YData', root.pos(2), 'ZData', 0, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0.6 0], 'MarkerEdgeColor', [0 0 0]);
    root.trajHandle = [];
    root.cost = 0;
    nodes = root;

    goalIdx = [];    
    goalNodes = [];
    normalTrajStyle = {'Color', [0 0 0], 'LineWidth', 1};
    goalTrajStyle = {'Color', 0.8 * [1 0 0], 'LineWidth', 2};
    % random number generator for iterations
    rng(0); % rng('shuffle');

    
    
    % do some initial iterations
    iterate(100);
    

    
    
    function iterate(iterations)
        for iter = 1:iterations
            if rand() < goalBias && isempty(goalNodes)
                sample = goal;
            else
                sample = [sizeX(1) + rand() * (sizeX(2) - sizeX(1)); ...
                          sizeY(1) + rand() * (sizeY(2) - sizeY(1))];
            end
            allNodePositions = [nodes.pos];
            distSq = sum((allNodePositions - repmat(sample, 1, size(nodes, 2))).^2);
            [minDistSq, minIdx] = min(distSq);                
            segmentLength = min(sqrt(minDistSq), maxExtensionDistance);
            nearPos = nodes(minIdx).pos;

            delta = sample - nearPos;        
            newPos = nearPos + segmentLength * delta / norm(delta);
            if ~collisionCheck(nearPos, newPos, obstacles)            
                newIdx = length(nodes) + 1;
                nodes(newIdx).pos = newPos;                
                nodes(newIdx).parent = minIdx;
                nodes(newIdx).cost = nodes(minIdx).cost + norm(newPos - nearPos);
                nodes(minIdx).children(end + 1) = newIdx;
                
                nodes(newIdx).trajHandle = line('Parent', ax, 'XData', [nearPos(1), newPos(1)], 'YData', [nearPos(2), newPos(2)], 'Color', [0 0 0]);                                
                if norm(newPos - goal) < goalDistance
                    goalNodes(end + 1) = newIdx;
                end
                
                if ~isempty(goalNodes)
                    goalCosts = [nodes(goalNodes).cost];
                    [~, bestGoalCostIdx] = min(goalCosts);
                    setGoal(goalNodes(bestGoalCostIdx));
                end
            end
        end
        titleText = sprintf('Number of nodes: %d', length(nodes));
        if ~isempty(goalIdx)
            titleText = [titleText sprintf(', best path cost (length) = %f\n', nodes(goalIdx).cost)];
        end
        title(titleText);
    end

    function reduceCostRecursive(idx, delta)
        nodes(idx).cost = nodes(idx).cost - delta;
        for iChild = nodes(idx).children
            reduceCostRecursive(iChild, delta);
        end
    end
    function setGoal(idx)
        if ~isempty(goalIdx)
            set([nodes.trajHandle], normalTrajStyle{:});
        end
        goalIdx = idx;
        if ~isempty(goalIdx)
            while ~isempty(idx)
                set(nodes(idx).trajHandle, goalTrajStyle{:});
                idx = nodes(idx).parent;
            end            
        end
    end
end

% obstacles = Nx4 array, each row = [x, y, w, h] of one rectangular
% obstacle
function b = collisionCheck(p, q, obstacles)    
    for i = 1:size(obstacles, 1)
        left = obstacles(i, 1);
        bot = obstacles(i, 2);
        w = obstacles(i, 3);
        h = obstacles(i, 4);
        right = obstacles(i, 1) + w;        
        top = obstacles(i, 2) + h;
        % check if bounding boxes intersect
        if p(1) <= left && q(1) <= left; continue; end
        if p(1) >= right && q(1) >= right; continue; end
        if p(2) <= bot && q(2) <= bot; continue; end
        if p(2) >= top && q(2) >= top; continue; end
        % check if either endpoint is inside the rectangle
        if p(1) > left && p(1) < right && p(2) > bot && p(2) < top
            b = true;
            return;
        end
        if q(1) > left && q(1) < right && q(2) > bot && q(2) < top
            b  = true;
            return;
        end
        % Check for edge intersections
        D = q - p;
        if D(1) ~= 0
            t = ([left, right] - p(1)) / D(1);
            r = (t * D(2) + p(2) - bot) / h;
            if any(t > 0 & t < 1 & r > 0 & r < 1)
                b = true; % intersection with left or right edge
                return;
            end
        end
        if D(2) ~= 0
            t = ([bot, top] - p(2)) / D(2);
            r = (t * D(1) + p(1) - left) / w;
            if any(t > 0 & t < 1 & r > 0 & r < 1)
                b = true; % intersection with top or bottom edge
                return;
            end
        end
    end
    b = false;
end
