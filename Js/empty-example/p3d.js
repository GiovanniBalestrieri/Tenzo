
var angle = PI/4
var z = -100; 
var pg;

var radius = 200;

var message = "tickle",
  font,
  bounds, // holds x, y, w, h of the text's bounding box
  fontsize = 60,
  x, y; // x and y coordinates of the text

var teapot;

function preload() {
  teapot = loadModel('teapot.obj',true);
}

function setup(){
	//var z = -100; 
	createCanvas(800,800,WEBGL);
	ortho(-width, width, height, -height/2, 0.1, 100);

	pg = createGraphics(256,256);

	

//	ambientLight(240,240,240); //even red light across our objects
	
}


function draw(){ 
	//ambientMaterial(250);
	background(100)

	pointLight(255, 255, 255, mouseX+width/2, mouseY+height/2, 0);


	noStroke();
	var dirY = (mouseY / height - 0.5) * 4;
	var dirX = (mouseX / width - 0.5) * 4;
	directionalLight(50, 204, 204, dirX, dirY, 1);
	//translate(-1.5 * radius, 0, 0);
	//sphere(radius);

	specularMaterial(250,25,40);
	sphere(100);



	model(teapot);	
    
	
	rotateX(PI/3)
	translate(width*0.0,-height*0.3,-400)

	box(100,100,100); //draws a box of width: 10, height: 20, and depth: 30
	translate(-width*0.0,height*0.3,100)
	cone(40, 100, 100);//d

/*
	translate(100,300,400);
	rotateX(angle);
	box();

	translate(100,100,400);
	rotateX(angle);

	box();
	translate(200,100,400);
	rotateY(angle);
	rotateZ(angle);
	box();

	translate(400,100,-100);
	rotate(PI/4, [1,1,0]);
  	box(20)
  	*/
  }

  function drawChildren(children,Dist) {
	var sec = PI/2;
	var frac = sec*1/children;
	push();
	rotateX(frac);
	translate(0,Dist,0);
	sphere(10)
	pop()
	push()
	rotateX(-frac);
	translate(0,Dist,0);
	sphere(10)
	pop()
  }