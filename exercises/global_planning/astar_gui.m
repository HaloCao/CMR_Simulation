function astar_gui()
    
    % ======= Parameters (free to modify) =================================
    
    mapPath = 'map_office.png';
    obstacleColor = [0 0 0];
    start = [50; 50];
    %goal = [100, 150];
    goal = [250, 80];
    useManualStepping = true;
    
    % ======= END: Parameters =============================================    
    
	% load map
	[imgData, clrMap] = imread(mapPath);
	if isempty(clrMap); 
		error('Map file is not an indexed-color image!');
	end

	obstacleColorIndex = find(sum(abs(clrMap - repmat(obstacleColor, size(clrMap, 1), 1)), 2) == 0, 1, 'first');
    if ~isempty(obstacleColorIndex)
        obstacleMap = (imgData == (obstacleColorIndex - 1));
    else
        warning('astar:noObstacle', 'Obstacle color not found in map image!');
        obstacleMap = false(size(imgData));
    end
	
    w = size(obstacleMap, 2);
    h = size(obstacleMap, 1);

	% initialize figure and graphic elements
	f = figure('Renderer', 'OpenGl', 'NumberTitle', 'off', 'Name', 'AStar Path Planning Example', 'Toolbar', 'figure');
	ax = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'DataAspectRatio', [1 1 1], 'Layer', 'top');
	
	hMap = image('Parent', ax, 'CData', 1 - repmat(double(obstacleMap), [1 1 3]), 'XData', [1 w] - 0.5, 'YData', [1 h] - 0.5);
	hPath = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [0.1 0.1 1], 'LineWidth', 2);
    hStart = patch('Parent', ax, 'XData', start(1) - 0.5, 'YData', start(2) - 0.5, 'EdgeColor', 'none', 'Marker', 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0 0.6 0], 'MarkerEdgeColor', [0 0 0]);	
    hGoal = patch('Parent', ax, 'XData', goal(1) - 0.5, 'YData', goal(2) - 0.5, 'EdgeColor', 'none', 'Marker', 'x', 'MarkerSize', 10, 'MarkerEdgeColor', [1 0 0], 'LineWidth', 2);
    hShade = patch('Parent', ax, 'XData', [0 w w 0], 'YData', [0 0 h h], 'FaceColor', [0 0 0], 'FaceAlpha', 0.5, 'Visible', 'off');
    hMessage = text('Parent', ax, 'String', '', 'Color', [1 0 0], 'FontWeight', 'bold', 'FontName', 'Verdana', 'FontUnits', 'points', 'FontSize', 16, ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Position', [w h] / 2);
    hHeapHead = line('Parent', ax, 'XData', [], 'YData', [], 'Color', [1 0.5 0], 'LineStyle', 'none', 'Marker', 's', 'MarkerSize', 7, 'HitTest', 'off');
	% add ui controls for manual stepping
    hManualCheck = uicontrol('Parent', f, 'style', 'checkbox', 'String', 'Manual Iterations:', 'Units', 'pixels', 'Position', [4, 8, 150, 16], 'Value', useManualStepping, 'Callback', @manualCheckChanged);
    hIterationButton = [0 0 0];    
    for i = 0:2
        nIterations = 10^i;
        hIterationButton(i + 1) = uicontrol('Parent', f, 'style', 'pushbutton', 'String', sprintf('%d', nIterations), 'Units', 'pixels', 'Position', [158 + i * 44, 4, 40, 24], ...
                                            'Callback', @(varargin)runPlanner(nIterations));
    end
    hAutoIterateButton = uicontrol('Parent', f, 'style', 'pushbutton', 'String', 'All', 'Units', 'pixels', 'Position', [158 + 3 * 44, 4, 40, 24], ...
                                   'Callback', @(varargin)runPlanner(inf));
    setManualButtonsEnabled(get(hManualCheck, 'Value'));
    
    
    planner = [];
    startPlanner();
    
    set(f, 'WindowButtonDownFcn', @mouseDown);
    set([hStart, hGoal], 'ButtonDownFcn', @mouseDown);
    
    % Initialize a new planning task (if start and/or goal position has changed)
    % This generates a new planner 'object'
    function startPlanner()
        planner = astar_planner(obstacleMap, start, goal);
        if ~get(hManualCheck, 'Value')
            runPlanner(inf);
        else runPlanner(0);
        end
    end
    % Run at most the given number of A* iterations and update the graphics according to the results
    function runPlanner(iterations)
        isManual = get(hManualCheck, 'Value');
        if iterations < inf
            [waypoints, errMsg, cellTypes, heapHead] = planner.run(iterations);
        else
            set(hShade, 'Visible', 'on');
            set(hMessage, 'String', 'Computing...');
            drawnow();

            [waypoints, errMsg, cellTypes, heapHead] = planner.run();

            set(hShade, 'Visible', 'off');        
        end

        set(hMap, 'CData', ind2rgb(cellTypes, [1 1 1; 0 0 0; 0.8 1 0.4; 0.7 0.7 0.7; 1 0.5 0]));
        set(hHeapHead, 'XData', heapHead(1, :) - 0.5, 'YData', heapHead(2, :) - 0.5);
        
        if ~isempty(waypoints)
            set(hPath, 'XData', waypoints(1, :) - 0.5, 'YData', waypoints(2, :) - 0.5);        
            set(hMessage, 'String', '');
        else
            set(hPath, 'XData', [], 'YData', []);
            set(hMessage, 'String', errMsg);
        end
        
        setManualButtonsEnabled(isManual && isempty(waypoints) && isempty(errMsg));        
    end

    % ======== GUI control callbacks and other internal functions =========
    
    function setManualButtonsEnabled(b)
        if b
            set([hIterationButton hAutoIterateButton], 'Enable', 'on');
        else set([hIterationButton hAutoIterateButton], 'Enable', 'off');
        end
    end
    function manualCheckChanged(varargin)
        isManual = get(hManualCheck, 'Value');
        setManualButtonsEnabled(isManual);
        if isManual; runPlanner(0);
        else runPlanner(inf);
        end
    end
    
    function mouseDown(h, varargin)        
        if h == hStart; pointerObj = 1;
        elseif h == hGoal; pointerObj = 2;
        else
            pos = getPointerPosition(ax);
            info = planner.getCellInfo(pos(1), pos(2));
            fprintf('Cell (%d, %d)\n%s\n - g_cost = %d\n - f_cost = %d\n', pos(1), pos(2), info.type, info.g_cost, info.f_cost);            
            return;
        end

        set(f, 'WindowButtonMotionFcn', @mouseMove, 'WindowButtonUpFcn', @mouseUp);
        mouseMove();

        function mouseMove(varargin)
            pos = getPointerPosition(ax);
            if pointerObj == 1
                start = pos;
                set(hStart, 'XData', start(1) - 0.5, 'YData', start(2) - 0.5);
            elseif pointerObj == 2
                goal = pos;
                set(hGoal, 'XData', goal(1) - 0.5, 'YData', goal(2) - 0.5);
            end
        end        
        function mouseUp(varargin)
            set(f, 'WindowButtonMotionFcn', [], 'WindowButtonUpFcn', []);
            startPlanner();
        end
    end
    function pos = getPointerPosition(ax)
        cp = get(ax, 'CurrentPoint');
        pos = round([cp(1, 1); cp(1, 2)] + 0.5);
        pos(pos < 1) = 1;
        if pos(1) > w; pos(1) = w; end
        if pos(2) > h; pos(2) = h; end        
    end
end

