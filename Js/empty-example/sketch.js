var x = 0;
var width = 600;
var heigth = 400;
var img;


function preload() {
  img = loadImage('test.jpg');
}

function setup() {
  var myCanvas = createCanvas(heigth, width);
  image(img, 0, 0);

  myCanvas.parent('myContainer');
  line(15, 25, 70, 90);

  drawingContext.shadowOffsetX = -1;
  drawingContext.shadowOffsetY = -1;
  drawingContext.shadowBlur = 10;
  drawingContext.shadowColor = "black";
  background(200);
}


function draw() {
  background(100);  
  image(img, 0, 0);

  ellipse(x, height/2, 20, 20);
  x = x + 1;  
}