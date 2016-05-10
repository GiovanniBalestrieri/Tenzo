% Copyright (c) 2016, The MathWorks, Inc.
classdef (ConstructOnLoad) ItemMovedEventData < event.EventData
    
   properties
      Index
      Type
   end
   
   methods
      function data = ItemMovedEventData(type, ind)
         data.Index = ind;
         data.Type = type;
      end
   end
end