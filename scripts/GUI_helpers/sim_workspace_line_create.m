function lineElements = sim_workspace_line_create(parent,position,signalName,params, deleteCallback)
    
    pad = 3;
    height = position(4);
    width = position(3);
    x = position(1);
    y = position(2);

    gap = 3;
    labelW = width*1/2;
    dropdownW = (width-labelW)*1/4;
    checkW = (width-labelW)*1/4;
    btnW = (width-labelW)*1/4;

    
    lineElements.label = uilabel(parent, ...
        "Text",signalName,'HorizontalAlignment','left', ...
        'Position',[x+gap, y, labelW,height]);

    if isstruct(params)
        fn = fieldnames(params);
        paramList = cellfun(@(f) sprintf('%s: %s', f, num2str(params.(f))), fn, 'UniformOutput', false);
    end

     lineElements.paramDropdown = uidropdown(parent, ...
        'Items', paramList, ...
        'Editable', 'off', ...
        'Tooltip', 'Click to view parameters', ...
        'Position', [x + labelW + gap, y, dropdownW, height-2*gap]);

      lineElements.isActive = uicheckbox(parent, ...
        "Text",'', ...
        'Position', [x + labelW + dropdownW + 2*gap, y, checkW, height], ...
        'Value', false, ...
        'ValueChangedFcn', @(src,~) setActiveSignal('active', signalName, src.Value));
    
    

    uiline = uilabel(parent, ...
        'Position', [x, y-2, width, 2], ...
        'BackgroundColor', [0.8 0.8 0.8], ...
        'Text', '');
    lineElements.separator = uiline;

    lineElements.deleteBtn = uibutton(parent, ...
        'Text', '✕', ...
        'FontWeight', 'bold', ...
        'Tooltip', 'Delete this signal', ...
        'Position', [x + labelW + dropdownW + 2*checkW + 2*gap, y + 1, btnW, height - 2], ...
        'ButtonPushedFcn', @(btn,~) deleteWorkspaceLine(lineElements,signalName,deleteCallback));
    
    

    lineElements.containerPos = position;
    lineElements.signalName = signalName;
end

function setActiveSignal(type, name, state)
    fprintf("Signal '%s' set as active_%s = %d\n", name, type, state);
end

function deleteWorkspaceLine(lineElements, signalName, deleteCallback)
    fields = fieldnames(lineElements);
    for i = 1:numel(fields)
        h = lineElements.(fields{i});
        if ishghandle(h)
            delete(h);
        end
    end
    if isa(deleteCallback, 'function_handle')
        deleteCallback(signalName);
    end
end