%environment setup
function DobotTM5Environment()
%Clear any open windows from previous outputs
close all
hold on
axis([-3, 3, -3, 3, 0, 4])
xlabel('x');
ylabel('y');
zlabel('z');

%Safety in design environment setup

PlaceObject('bluetable.ply', [0,-0.5,0]);
PlaceObject('fireExtinguisher.ply', [-1.5,-2.25,0]);
PlaceObject('emergencyStopButton.ply', [0,-1,0.8]);
PlaceObject('barriercombined.ply', [-1.5,-1.5,0]);
PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [0,2.6,0]);
PlaceObject('bookcaseTwoShelvesrotated.ply', [-2.6,0,0]);
PlaceObject('brick.ply', [1.02,-1,0.9]); %adjusted brick
PlaceObject('brick.ply', [1,-1.02,0.98]);%adjusted brick
PlaceObject('brick.ply', [1,-1,1.06]);
PlaceObject('brick.ply', [1,-0.75,0.9]);
PlaceObject('brick.ply', [1,-0.75,0.98]);
PlaceObject('brick.ply', [1,-0.75,1.06]);
PlaceObject('brick.ply', [1,-1.25,0.9]);
PlaceObject('brick.ply', [1,-1.25,0.98]);
PlaceObject('brick.ply', [1,-1.25,1.06]);

PlaceObject('baby.ply', [-2.5,2.5,0]);
%PlaceObject('personFemaleBusiness.ply', [2.5,2.5,0]);
PlaceObject('personMaleCasual.ply', [2.5,-2.5,0]);
PlaceObject('personMaleConstruction.ply', [-2.25,-2.5,0]);
PlaceObject('personMaleOld.ply', [1.75,0.75,0]);
PlaceObject('chair.ply', [2.5,2.5,0]);
surf([-3,-3;3,3],[-3,3;-3,3],[0.02,0.02;0.02,0.02],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
%surf([-3,-3;3,3],[ y, z, 'CData', CData, 'FaceColor', FaceColor);
%%
%Load in the movement function
LinearUR3movement()
%r = LinearUR3movement(transl(0.5, 0, 0.85))
end


% qPath = jtraj(r.model.qlim(:,1)',r.model.qlim(:,2)',600);
%     for i = 1:length(qPath)
%         r.model.animate(qPath(i,:))
%         pause(0)
%         drawnow();
%     end
%     %hold on



% for repetition = 1:numRepetitions
%     qPath = jtraj(r.model.qlim(:,1)',r.model.qlim(:,2)',200);
% 
%     for i = 1:length(qPath)
%     r.model.animate(qPath(i,:))
%     pause(0)
%     end

% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [2,-2,0]);
% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [0,-2,0]);
% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [-2,-2,0]);
% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [-2,4,0]);
% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [0,4,0]);
% barrier = PlaceObject('barrier1.5x0.2x1m.ply', [2,4,0]);