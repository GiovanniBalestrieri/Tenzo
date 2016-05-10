% Copyright (c) 2016, The MathWorks, Inc.
classdef (ConstructOnLoad) ItemAddedEventData < event.EventData
    
   properties
      Type
   end
   
   methods
      function data = ItemAddedEventData(type)
         data.Type = type;
      end
   end
end