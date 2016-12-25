//draw a spinning teapot
var teapot;

function setup(){
  createCanvas(200, 200, WEBGL);

  teapot = loadModel('teapot.obj',true);
}

function draw(){
  background(200);
  rotateX(frameCount * 0.01);
  rotateY(frameCount * 0.01);
  model(teapot);
}