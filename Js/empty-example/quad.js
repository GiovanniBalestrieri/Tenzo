//draw a spinning teapot
var teapot;
var quad;
var xCam,yCam,zCam;
var img; 
var cubeTexture;
var drawPlane = false;
var drawTeapot = false;
var quadPoseX = 0;
var quadPoseY = -500;
var quadPoseZ = 200;
var xSkip = 50;
var ySkip = 50;

var value = 0;

function setup(){
	frameRate(20);
	createCanvas(500, 500, WEBGL);
	quad = loadModel('/home/userk/development/github/Tenzo/Js/empty-example/assets/quad/MQ-27.obj',true);
	teapot = loadModel('/home/userk/development/github/Tenzo/Js/empty-example/assets/teapot/teapot.obj',true);

	img = loadImage('assets/quad/Tan.jpg');
	cubeTexture = loadImage('assets/quad/1.png');

	xCam = 0;
	yCam = -100;
	zCam = 400;
}

function draw(){
	background(111);
	//zCam++;
	translate(0,50,0)
	createGrid(4,5)

	box()

	camera(xCam,yCam,zCam)
	orbitControl();
	
	//pointLight(250, 250, 250, locX, locY, 0);
	//ambientMaterial(250);

	// Draw teapot
	push()
	if (drawTeapot){
		rotateX(PI)
			translate(0,50,0)
			fill(200,10,80);
			//rotateX(frameCount * 0.01);
			//rotateY(frameCount * 0.01);
			model(teapot);
		pop()
	}


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
		translate(quadPoseX,quadPoseY,quadPoseZ)
		//rotateX(PI)
		//spotLight(255, 0, 0, width/2, height/2, 400, 0, 0, -1, PI/4, 2);
		noFill();
		//otateX(frameCount * 0.01);
		rotateY(frameCount * 0.01);
		texture(img);
		model(quad);
	pop()
}

/**
 * 
 */
function keyPressed() {

  if(keyCode == LEFT_ARROW)
    xCam-=xSkip;

  if (keyCode ==RIGHT_ARROW)
    xCam+=xSkip;

  if (keyCode ==UP_ARROW)
    zCam-=ySkip;

  if (keyCode ==DOWN_ARROW)
    zCam+=ySkip;


  if (keyCode ==CONTROL)
    yCam+=ySkip;


  if (keyCode ==SHIFT)
    yCam-=ySkip;

}


/**
 * Creates a grid

 **/
function createGrid(dim, unitSize){

	//translate(0,0,0); 
	push();

	    console.log("F")
	    translate(0,0,0)
		texture(cubeTexture);
	    box(1000,1,1000)
	    /*
		var i,j;
	    for (var i = 0; i<dim; i++) {
	        //console.log("A")
	      for (j = 0; j<dim; j++) {
	      	var aa = (i+j)%2
	        
	        
	        //if ((i+j)%2 === 0) {
	        	//console.log("i: " , i , " j: " ,j)
	        	translate( i*dim,0, + j*dim);
	        	//noTexture();
				texture(cubeTexture);
				
	        	box(100,10,100);
	        	//rect(i*unitSize, j*unitSize, unitSize, unitSize);
	      //}
	    }
	    */
	
	pop();
}