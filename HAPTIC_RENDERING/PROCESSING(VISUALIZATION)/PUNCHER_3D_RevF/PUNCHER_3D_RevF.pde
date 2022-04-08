// PUNCHER VISUALIZATION
// By Øystein Bjelland, CPS-Lab, NTNU
// Changelog Rev F: Trying to implement camera 3D-orient from this example (https://discourse.processing.org/t/camera-rotation-in-3d-using-mouse/20563) 

import processing.serial.*;
import processing.opengl.*;

Serial myPort; // Create object from Serial class

//General
PShape NTNU;

//3D-view setup
float triggerAngle = 0;
public float angleChange = 0;
float previousTriggerAngle = triggerAngle;
  
PShape mainBody;
PShape trigger;
PShape puncher;
    
PShape meniscus;
PShape bone;
PShape fibula;


//*****************************

//Rubber Band
int DOF = 10;
String val; //Data received from the serial port


float[] deformedMesh = new float[DOF];
float[] node_x = new float[DOF+1];
float[] staticMesh = new float[DOF];

float x_offset = 0;
float y_offset = 0; // =height/2;

float meniscusThickness = 30;
float nodeDistance = 50;
float toolSize = 18;
float toolYPos;
float toolXPos = 1.5;
int toolXPosIndex;

int startTime;
int time;

float a = 0.5;
float b = 2;

//Anatomy transformation parameters
float scaleAnatomy = 0.08;
float translateAnatomyX = -320;
float translateAnatomyY = -220;
float translateAnatomyZ = -140;
float rotateAnatomyX = 20;
float rotateAnatomyY = -100;
float rotateAnatomyZ = -80;

//******************************

//Camera stuff
int grid = 100;
PVector bpos = new PVector();
float bsize = grid;
float bspeedX, bspeedY, bspeedZ;
boolean input1, input2, input3, input4;
float cameraRotateX;
float cameraRotateY;
float cameraSpeed;
int gridCount = 50;
PVector pos, speed;
float accelMag;



void setup() {
    
    //General
    //size(1200, 800, P3D);
    fullScreen(P3D);
    background(0);
    stroke(255);
    strokeWeight(5);
    
    //Camera stuff
    cameraSpeed = TWO_PI / width;
    cameraRotateY = -PI/6;
    pos = new PVector();
    speed = new PVector();
    accelMag = 2;
    
    String portName = Serial.list()[1];  // Put correct port number here
    myPort = new Serial(this, portName, 115200);  //Put correct baud rate at last entry!
    myPort.bufferUntil('\n');
   
     
    //3D-view stuff
    mainBody = loadShape("ACUFEX_PUNCH_MAIN_BODY.obj");
    trigger = loadShape("TRIGGER_NEW_CSYS.obj");
    puncher = loadShape("PUNCHER_OBJ.obj");
    //meniscus = loadShape("meniscus.obj");
    //bone = loadShape("bone_medullar.obj");
    bone = loadShape("tibia.obj");
    fibula = loadShape("fibula.obj");
    meniscus = loadShape("MENISCUS_BUCKET_HANDLE.obj");
    
    
    shape(mainBody, 80, 0);
    mainBody.rotateX(PI/2);
    mainBody.translate(-23., 49, 0);
    
    shape(trigger, 0, 0);
    trigger.rotateZ(radians(20));
    
    shape(puncher, 0, 0);
    puncher.rotateZ(radians(35));
    puncher.translate(-202,-12,0);
   
    shape(meniscus, 0, 0);
    meniscus.scale(scaleAnatomy);   
    meniscus.rotateY(radians(rotateAnatomyY));   
    meniscus.rotateZ(radians(rotateAnatomyZ));
    meniscus.rotateX(radians(rotateAnatomyX));
    meniscus.translate(translateAnatomyX,translateAnatomyY,translateAnatomyZ);
    meniscus.setFill(color(255,0,0));
     
    shape(bone, 0, 0);
    bone.scale(scaleAnatomy);
    bone.rotateY(radians(rotateAnatomyY));
    bone.rotateZ(radians(rotateAnatomyZ));
    bone.rotateX(radians(rotateAnatomyX));
    bone.translate(translateAnatomyX,translateAnatomyY,translateAnatomyZ);
    bone.setFill(color(200,200,200));
  
    shape(fibula, 0, 0);
    fibula.scale(scaleAnatomy); 
    fibula.rotateY(radians(rotateAnatomyY));
    fibula.rotateZ(radians(rotateAnatomyZ));
    fibula.rotateX(radians(rotateAnatomyX));
    fibula.translate(translateAnatomyX,translateAnatomyY,translateAnatomyZ);
    fibula.setFill(color(200,200,200));
       
    //Rubberband stuff
    stroke(255);
    strokeWeight(1); 
  
    //x_offset = width/2;
    //y_offset = height/2.22;
    
    x_offset = -400;
    y_offset = 0;
     
    for(int i = 0; i < DOF; i++)
    {
      deformedMesh[i] = 0; 
      //staticMesh[i] = a + ((b-a)/DOF)*i; 
    }  
    
    for(int i = 0; i < DOF + 1; i++)
        node_x[i] = a + ((b-a)/DOF)*i;  
    
    
    //Find index of tool position
    for(int i = 0; i < DOF; i++)
    {
      if ((toolXPos >= node_x[i]) && (toolXPos < node_x[i+1])){
        toolXPosIndex = i;
      }
    }
}


  
void draw() {
    
    //General view stuff
    background(0);
    lights();
    

    rotateX(cameraRotateY);
    rotateY(cameraRotateX);


    // Get tool position and map to 3D- and 2D-views
    toolYPos = deformedMesh[toolXPosIndex];
    triggerAngle = toolYPos*0.8; // Shortcut kinematic transformation from position to angle.
    angleChange = -(triggerAngle - previousTriggerAngle);
    previousTriggerAngle = triggerAngle; 
    
    
    // Rubberband stuff
    strokeWeight(5);
    
    pushMatrix();
    //translate(width/3, height / 1.5); //Translate 3D-view to middle of view port before further transformations  
    translate(width/2, height / 2); //Translate 3D-view to middle of view port before further transformations   
       
    //Draw nodes (only display deformed mesh if there is collision beteween tool and mesh)
    if (toolYPos < 0){
           for(int i = 0; i < DOF; i++)
            {
              point(node_x[i] * nodeDistance + x_offset, staticMesh[i]+y_offset); 
             // point(node_x[i] * nodeDistance + x_offset, y_offset + meniscusThickness);
            }
    }else{
           for(int i = 0; i < DOF; i++)
            {
              point(node_x[i] * nodeDistance + x_offset, deformedMesh[i]+y_offset); 
             // point(node_x[i] * nodeDistance + x_offset, y_offset + meniscusThickness);
            }
    }
    
  
    
    //3D-view stuff

    drawPunch(); //Draw punch with moving parts
    
    shape(meniscus, 0, 0);
    shape(bone, 0, 0);
    shape(fibula, 0, 0);
    //shape(meniscus2, 0, 0);
            
    popMatrix();               
    
    //CAMERA STUFF
    PVector accel = getMovementDir().rotate(cameraRotateX).mult(accelMag);
    speed.add(accel);
    pos.add(speed);
    speed.mult(0.9);  
    translate(0, height/2+bsize/2);

}



//Functions for drawing puncher
  
  void drawPunch(){
    shape(mainBody, 80, 0);
    drawTrigger();
    drawPuncher();
    
  }
  
  void drawTrigger(){
   pushMatrix();
   trigger.rotateZ(radians(angleChange));
   shape(trigger, 0, 0); 
   popMatrix();
   //println(triggerAngle);    
   delay(80); //Strange behavior from program requires delay here to render moving parts
  }
  
  void drawPuncher(){
    pushMatrix();
    puncher.translate(202,12,0);
    puncher.rotateZ(radians(angleChange));
    puncher.translate(-202,-12,0);
    shape(puncher,0,0);
    popMatrix();
  }
  

  
// Arduino communication
void serialEvent(Serial p){
      
    if ( p.available() > 0) 
  {  // If data is available,
  val = p.readStringUntil('\n');         // read it and store it in val
  } 
  
  if(val == null)
    return;
    
  String sp[] = split(val, ",");
  if(sp.length < DOF)
    return;
  //val=myPort.readStringUntil('\n');  
  
  //time = second()-startTime; //Elapsed time from start in seconds
  
    //println(val); //Uncomment to see mesh pos in serial monitor
  //deformedMesh = float(split(val, ","));
    for (int i = 0; i<DOF; i++){
      deformedMesh[i] = float(sp[i]);
    }
  
}


//3D camera functions

void mouseClicked()
{
  println("bpos.x" + bpos.x);
  println("bpos.y" + bpos.y);
  println("bpos.z" + bpos.z);
}

void mouseMoved() {
  cameraRotateX += (mouseX - pmouseX) * cameraSpeed;
  cameraRotateY += (pmouseY - mouseY) * cameraSpeed;
  cameraRotateY = constrain(cameraRotateY, -HALF_PI, 0);
}

PVector getMovementDir() {
  return pressedDir.copy().normalize();
}

boolean wPressed, sPressed, aPressed, dPressed;
PVector pressedDir = new PVector();

void keyPressed() {
  switch(key) {
  case 'w':
    wPressed = true;
    pressedDir.y = -1;
    break;
  case 's':
    sPressed = true;
    pressedDir.y = 1;
    break;
  case 'a':
    aPressed = true;
    pressedDir.x = -1;
    break;
  case 'd':
    dPressed = true;
    pressedDir.x = 1;
    break;
  }
}

void keyReleased() {
  switch(key) {
  case 'w':
    wPressed = false;
    pressedDir.y = sPressed ? 1 : 0;
    break;
  case 's':
    sPressed = false;
    pressedDir.y = wPressed ? -1 : 0;
    break;
  case 'a':
    aPressed = false;
    pressedDir.x = dPressed ? 1 : 0;
    break;
  case 'd':
    dPressed = false;
    pressedDir.x = aPressed ? -1 : 0;
    break;
  }
}
