% Copyright 2016 The MathWorks, Inc.
classdef RayCasterU < handle
    
    properties(Access=private)
        
        vertices
        edges
        
    end
    
    properties(GetAccess=public,SetAccess=private)
        
        VisionPolygonX
        VisionPolygonY
        
    end
    
    properties(Access=public)
        
        ViewingPosition = [0 0]
        
    end
    
    %Helpers for indexing
    properties(Constant)
        
        X = 1;
        Y = 2;
        relX = 3;
        relY = 4;
        relT = 5;
        dist = 6;
        Left = 1;
        Right = 2;
        
    end
    
    methods
        
        function obj = RayCasterU(v, e)
            
            obj.vertices = [v zeros(length(v),4)];
            obj.edges = [e zeros(length(e),4)];
            obj.VisionPolygonX = zeros(length(v), 1);
            obj.VisionPolygonY = zeros(length(v), 1);
            
        end
        
    end
    
    methods(Access=public)
        
        function CastRays(obj)
            % Cast rays
            
            % Compute relative cartesian coordinates
            obj.vertices(:,RayCasterU.relX) = obj.vertices(:,RayCasterU.X) -...
                obj.ViewingPosition(RayCasterU.X);
            
            obj.vertices(:,RayCasterU.relY) = obj.vertices(:,RayCasterU.Y) -...
                obj.ViewingPosition(RayCasterU.Y);
            
            
            % Compute relative polar coordinates
            [obj.vertices(:,RayCasterU.relT), obj.vertices(:,RayCasterU.dist)] =...
                cart2pol(obj.vertices(:,RayCasterU.relX), obj.vertices(:,RayCasterU.relY));
            
            obj.vertices(:,RayCasterU.relT) = mod(obj.vertices(:,RayCasterU.relT), 2*pi);
            
            
            % Sort rows by angle and get reverse permutation
            [obj.vertices, ind] = sortrows(obj.vertices, RayCasterU.relT);
            
            [~, ind] = sort(ind, 'ascend');
            
            
            % Modify the edges indices accordingly
            obj.edges(:,RayCasterU.Left) = ind(obj.edges(:,RayCasterU.Left));
            
            obj.edges(:,RayCasterU.Right) = ind(obj.edges(:,RayCasterU.Right));
            
            
            % Ensure that the left vertex of each edge is on the left
            Tdiff = obj.vertices(obj.edges(:,RayCasterU.Left),RayCasterU.relT) -...
                obj.vertices(obj.edges(:,RayCasterU.Right),RayCasterU.relT);
            
            ind = xor(Tdiff < 0, abs(Tdiff)>pi);
            
            obj.edges(ind, [RayCasterU.Left RayCasterU.Right]) =...
                obj.edges(ind, [RayCasterU.Right RayCasterU.Left]);
            
            
            % Loop over each vertex and get the left vertex of the
            % right triangle and the right edge of the left triangle
            for ray = 1:size(obj.vertices,1)
                
                % Compute intersection coordinates and find edges that
                % cross the ray
                [~, ~, interX, interY, ua, ub] = lineSegmentsIntersection(...
                    obj.ViewingPosition(1), obj.ViewingPosition(2),...
                    obj.vertices(ray,1), obj.vertices(ray,2),...
                    obj.vertices(obj.edges(:,1),1),...
                    obj.vertices(obj.edges(:,1),2),...
                    obj.vertices(obj.edges(:,2),1),...
                    obj.vertices(obj.edges(:,2),2));
                intersects = find((ua >= 0) & (ub >= 0) & (ub <= 1));
                
                interX = interX(intersects);
                interY = interY(intersects);
                
                
                % Sort the intersections (coordinates and index of the
                % edges) by distance to viewpoint
                dist2vp = ua(intersects) * obj.vertices(ray,RayCasterU.dist);

                [~,ind] = sort(dist2vp, 'ascend');
                
                intersects = intersects(ind);
                
                % Pick the first intersection, with the exception that it
                % must not be a right edge for the left triangle or a left
                % edge for the right triangle. Assign the result
                leftInd = find(obj.edges(intersects,RayCasterU.Right) ~= ray, 1, 'first');
                rightInd = find(obj.edges(intersects,RayCasterU.Left) ~= ray, 1, 'first');
                
                if isempty(leftInd)
                    obj.VisionPolygonX(2*ray-1) = obj.ViewingPosition(1);
                    obj.VisionPolygonY(2*ray-1) = obj.ViewingPosition(2);   
                else
                    obj.VisionPolygonX(2*ray-1) = interX(ind(leftInd));
                    obj.VisionPolygonY(2*ray-1) = interY(ind(leftInd));                    
                end
                
                if isempty(rightInd)                    
                    obj.VisionPolygonX(2*ray) = obj.ViewingPosition(1);
                    obj.VisionPolygonY(2*ray) = obj.ViewingPosition(2);
                else                    
                    obj.VisionPolygonX(2*ray) = interX(ind(rightInd));
                    obj.VisionPolygonY(2*ray) = interY(ind(rightInd));
                end
                
            end
            
            
        end
        
    end

    
end