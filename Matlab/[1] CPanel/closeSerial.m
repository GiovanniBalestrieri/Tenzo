clc
clear all
fwrite(s,'C');
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all
clc 
disp('Serial port Closed');