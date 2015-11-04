%% ===============================Object===================================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% October 28, 2015
%
% Class definition for the Object Object. Gives physical properties to an
% object that can be used in simulation.

classdef Object
    properties
        Name;
        type;
        states;
        mass;
        dims;
        color;
        props;
    end
    
    methods
        % Creates the Object object from the input obj struct.
        function Object = Object(obj)
            Object.Name = (obj.name); 
            Object.type = (obj.type);
            Object.states = (obj.states); 
            Object.mass = (obj.mass); 
            Object.dims = (obj.dims);
            Object.color = (obj.color);
            Object.props = (obj.props);
        end
    end
end