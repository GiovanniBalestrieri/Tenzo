%% Matlab tcp socket communcation

t = tcpclient('192.168.1.33', 8888, 'Timeout',20);

data = (1:10)

write(t,data);

if (t.BytesAvailable>0)
    try
        bytes = read(t)
        str = native2unicode(bytes);
        disp(str);
    catch ME
        rethrow(ME);
    end   
end