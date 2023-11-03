classdef OmronTM5 < RobotBaseClass
    %% APplying CAD model to robot

    properties(Access = public)              
        plyFileNameStem = 'TM5'; 
    end
    
    methods (Access = public) 
%% Define robot Function 
        function self = OmronTM5(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the robot links, DH parameters
            link(1) = Link('d',0.146,    'a',0,      'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-270),deg2rad(270)]);
            link(2) = Link('d',0,         'a',-0.329,  'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-180),deg2rad(180)]);
            link(3) = Link('d',0,         'a',-0.3115,'alpha',0,'offset',0,'qlim',[deg2rad(-155),deg2rad(155)]);
            link(4) = Link('d',0.106,    'a',0,      'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-180),deg2rad(180)]);
            link(5) = Link('d',0.1132,   'a',0,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
            link(6) = Link('d',0.05,     'a',0,      'alpha',0,'offset',3.14,'qlim',[deg2rad(-270),deg2rad(270)]);
            
            
            % Incorporate joint limits
%             link(1).qlim = [-270 270]*pi/180;
%             link(2).qlim = [-180 180]*pi/180;
%             link(3).qlim = [-155 155]*pi/180;
%             link(4).qlim = [-180 180]*pi/180;
%             link(5).qlim = [-180 180]*pi/180;
%             link(6).qlim = [-270 270]*pi/180;
        
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end