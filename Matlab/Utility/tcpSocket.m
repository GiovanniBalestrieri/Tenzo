%% Matlab tcp socket communcation

t = tcpclient('192.168.43.79', 8889, 'Timeout',20);

data = (1:10)


write(t,data);
while true
    if (t.BytesAvailable>0)
        try
            bytes = read(t)
            str = native2unicode(bytes);
            disp(str);
        catch ME
            rethrow(ME);
        end   
    end
end