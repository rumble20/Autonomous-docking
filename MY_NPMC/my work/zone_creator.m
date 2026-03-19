%% Load your map file first
load('helsinki_harbour_UPDATED.mat');  
%% Plot existing map
figure('Name', 'Zone Creator', 'NumberTitle', 'off');
hold on;
for kk = 1:length(map.polygons)
    patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
          'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1)
end
axis equal;
title('Click to add zones. Press ENTER when done with each zone.');

%% Add new zones interactively
addMore = true;
zonesAdded = 0;

while addMore
    disp('Click points to define a new forbidden zone.');
    disp('Press ENTER when polygon is complete.');

    [Y_coords, X_coords] = ginput();
    
    if length(X_coords) >= 3  % Need at least 3 points for a polygon
        X_coords(end+1) = X_coords(1);
        Y_coords(end+1) = Y_coords(1);
        
        % Add to map structure
        n = length(map.polygons);
        map.polygons(n+1).X = X_coords';
        map.polygons(n+1).Y = Y_coords';
        
        % Plot the new zone
        patch(Y_coords, X_coords, 'k', ...
              'FaceColor', [0.2 0.2 0.9], 'FaceAlpha', 0.3)
        
        zonesAdded = zonesAdded + 1;
        disp(['Zone ' num2str(zonesAdded) ' added successfully!']);
    else
        disp('Need at least 3 points. Zone not added.');
    end
    
    answer = input('Add another zone? (y/n): ', 's');
    addMore = strcmpi(answer, 'y');
end

%% Save updated map
if zonesAdded > 0
    saveFile = input('Save filename (without .mat): ', 's');
    if isempty(saveFile)
        saveFile = 'map_updated';
    end
    save([saveFile '.mat'], 'map');
    disp(['Map saved as: ' saveFile '.mat']);
    disp(['Total zones now: ' num2str(length(map.polygons))]);
else
    disp('No zones added. Map not saved.');
end

disp('Done!');
