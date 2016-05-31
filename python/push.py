import httplib, urllib
params = urllib.urlencode({'spam': 1, 'eggs': 2, 'bacon': 0})
headers = {"Content-type": "application/x-www-form-urlencoded", "Accept": "text/plain"}
conn = httplib.HTTPConnection("musi-cal.mojam.com:80")
conn.request("POST", "/cgi-bin/query", params, headers)
response = conn.getresponse()
print response.status, response.reason
#output: 200 OK
data = response.read()
conn.close()

httpServ = httplib.HTTPConnection("www.userk.co.uk/accenture/php/init.php", 80)
httpServ.connect()

def printText(txt):
    lines = txt.split('\n')
    for line in lines:
        print line.strip()

x=1
y=2
z=3

httpServ.request('POST', '/cgi_form.cgi', 'x=%s&y=%s&z=%s' % (x,y,z))

response = httpServ.getresponse()
if response.status == httplib.OK:
    print "Output from CGI request"
    printText (response.read())

httpServ.close()

