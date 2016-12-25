//draw a spinning teapot
var teapot;
var quad;
var xCam,yCam,zCam;
var img; 
var drawPlane = false;

function setup(){
	createCanvas(500, 500, WEBGL);
	quad = loadModel('/home/userk/development/github/Tenzo/Js/empty-example/assets/quad/MQ-27.obj',true);
	teapot = loadModel('/home/userk/development/github/Tenzo/Js/empty-example/assets/teapot/teapot.obj',true);

	img = loadImage('assets/quad/Tan.jpg');

	var fov = 60 / 180 * PI;
	var cameraZ = (height/2.0) / tan(fov/2.0);
	perspective(60 / 180 * PI, width/height, cameraZ * 0.1, cameraZ * 10);

	xCam = 0;
	yCam = -100;
	zCam = 100;
}

function draw(){
	background(100);
 	orbitControl();
	zCam++;

	//camera(xCam,yCam,zCam)

	// Draw teapot
	push()
	rotateX(PI)
		translate(0,50,0)
		fill(200,10,80);
		//rotateX(frameCount * 0.01);
		//rotateY(frameCount * 0.01);
		model(teapot);
	pop()


	if ( drawPlane) {
		push()
			translate(0,0,0)

			rotateX(PI/2);
			fill(0)
			//rotateY(frameCount * 0.01);
			plane(500, 10000);

			noFill();
		pop()
	}

	//Draw quad
	push()
		translate(width/5,0,200)
		//rotateX(PI)
		//spotLight(255, 0, 0, width/2, height/2, 400, 0, 0, -1, PI/4, 2);
		noFill();
		//otateX(frameCount * 0.01);
		rotateY(frameCount * 0.01);
		texture(img);
		model(quad);
	pop()
}