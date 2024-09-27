function environment ()

    clf;
    view(3)
    axis equal
    axis auto
    hold on 

    % Place floor
    surf([ 0,0; 1.5,1.5] ...
        ,[-1.6,0.6;-1.6,0.6] ...
        ,[-0.7971,-0.7971;-0.7971,-0.7971] ... %height of floor
        ,'CData',imread('floor.png') ...
        ,'FaceColor','texturemap');  

    % Place wall (back)
    surf([1.5, 1.5;0,0] ...
        ,[0.61,0.61;0.61,0.61] ...
        ,[-0.7971,1;-0.7971,1] ... %height of floor
        ,'CData',imread('wall.png') ...
        ,'FaceColor','texturemap');   

    % Place wall (right)
    surf([1.5, 1.5;1.5, 1.5] ...
        ,[-1.6,-1.6;0.61,0.61] ...
        ,[-0.7971,1;-0.7971,1] ... %height of floor
        ,'CData',imread('wall.png') ...
        ,'FaceColor','texturemap');  

    % Place plyFiles
    PlaceObject('stillEnvironment.ply',[0,0,-0.0427]); % z=0 is where the robots stand
    PlaceObject('eStop.ply',[1.5,-0.6,0.1]);     
    PlaceObject('fireExtinguisher.ply', [1.3, -0.1, -0.7971]);   
    PlaceObject('lightCurtain.ply', [0, -1, -0.7971]);
    PlaceObject('lightCurtain.ply', [1.45, -1, -0.7971]);
    PlaceObject('piggy.ply', [0.8, -1.3, -0.7971]);    
    plotLightCurtain();
 
end