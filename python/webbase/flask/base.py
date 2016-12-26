from flask import Flask, url_for, request, Response,json

app = Flask(__name__)

@app.route('/')
def api_root():
    return 'Welcome'

@app.route('/hello', methods = ['GET'])
def api_hello():
    data = {
        'hello'  : 'world',
        'number' : 3
    }
    js = json.dumps(data)

    resp = Response(js, status=200, mimetype='application/json')
    resp.headers['Link'] = 'http://luisrei.com'

    return resp

@app.route('/articles')
def api_articles():
    return 'List of ' + url_for('api_articles')

@app.route('/articles/<int:articleid>')
def api_article(articleid):
    return 'You are reading ' + articleid

@app.route('/hi')
def api_hi():
    if 'name' in request.args:
	return 'Hello' + request.args['name']
    else:
        return 'Hello Gepp'

@app.route('/echo', methods = ['GET', 'POST', 'PATCH', 'PUT', 'DELETE'])
def api_echo():
    if request.method == 'GET':
        return "ECHO: GET\n"

    elif request.method == 'POST':
        return "ECHO: POST\n"

    elif request.method == 'PATCH':
        return "ECHO: PACTH\n"

    elif request.method == 'PUT':
        return "ECHO: PUT\n"

    elif request.method == 'DELETE':
        return "ECHO: DELETE"



if __name__ == '__main__':
    app.run()

