% Copyright (c) 2016, The MathWorks, Inc.
classdef (ConstructOnLoad) ArenaModelEventData < event.EventData
    
   properties
      Index
      Type
   end
   
   methods
      function data = ArenaModelEventData(type, ind)
         data.Index = ind;
         data.Type = type;
      end
   end
end