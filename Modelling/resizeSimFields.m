function resizedSimOut = resizeSimFields(simOut)
    % Get all elements of the Simulink.SimulationOutput object
    elements = simOut.who;

    % Initialize an empty struct to store the resized data
    resizedSimOut = struct();
    
    len_tout = length(simOut.('tout'));
    
    % Loop through each element and apply the resizing function
    for i = 1:length(elements)
        elementName = elements{i};

        % Retrieve data using dynamic field referencing
        data = simOut.(elementName);
        
        % Check if the data is a matrix or array that needs resizing
        if size(data, 1) ~= len_tout
            % Example resizing function (replace with your resizing code)
            resizedSimOut.(elementName) = reshape(data, size(data, 1), len_tout)';
        else
            % If data doesn't need resizing, copy it as is
            resizedSimOut.(elementName) = data;
        end
    end
end