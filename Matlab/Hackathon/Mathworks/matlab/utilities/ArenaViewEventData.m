% Copyright (c) 2016, The MathWorks, Inc.
classdef (ConstructOnLoad) ArenaViewEventData < event.EventData
    
   properties
      Index
      Type
   end
   
   methods
      function data = ArenaViewEventData(type, ind)
         data.Index = ind;
         data.Type = type;
      end
   end
end