import controlP5.*;
import meter.*;
import processing.serial.*;

Serial port; // Define a port
ControlP5 cp5; // Create ControlP5 object
int[] serialInArray = new int[4]; // Where we'll put what we receive
int serialCount = 0;
boolean firstContact = false;        // Whether we've heard from the microcontroller
int slot, ultra, force; 
Meter m1, m2;
int[] serialOutArray = new int[7]; // where we'll put what we send [ServoPos, DCPos, DCVel, StepperPos]
boolean gui = true;

void setup(){
  // First we need to create a empty window
  size(1200, 700); // Size of the window (width, height)
  background(0, 0, 0); // Background color of window (R,G,B)
  
  // Create new port
  port = new Serial(this, "/dev/cu.usbmodem14301", 9600); //name of the port would be different for linux
  
  // ControlP5 slider
  cp5 = new ControlP5(this);
  
  cp5.addKnob("ServoPos",10,170,10,50,420,200);
  cp5.addKnob("DCPos",0,360,0,350,420,200);
  cp5.addKnob("DCVel",0,58,0,650,420,200).setShowAngleRange(true);
  cp5.addKnob("StepperPos",0,1080,0,950,420,200);
  
  cp5.addToggle("GUI-Sensor").setValue(false).setPosition(75,30).setSize(50,20).setMode(ControlP5.SWITCH);
  
  // FLEX METER
  m1 = new Meter(this, 200, 80);  // here 200, 80 are x and y coordinates of meter's upper left corner
  m1.setTitleFontSize(20);
  m1.setTitleFontName("Monospaced");
  m1.setTitle("Flex Sensor (g)");
  
  // Change meter scale values
  String[] scaleLabels = {"0", "300", "600", "900", "1200", "1500"};
  m1.setScaleLabels(scaleLabels);
  m1.setScaleFontSize(18);
  m1.setScaleFontName("Monospaced");
  m1.setScaleFontColor(color(200, 30, 70));
  
  // We can also display the value of meter
  m1.setDisplayDigitalMeterValue(true);
  
  // modifications so our meter looks nice
  m1.setArcColor(color(141, 113, 178));
  m1.setArcThickness(15);
  m1.setMaxScaleValue(1500);
  m1.setMinInputSignal(0); 
  m1.setMaxInputSignal(1500);
  m1.setNeedleThickness(3);
  
  // ULTRASONIC METER, take some refference from first meter
  int mx = m1.getMeterX(); // x coordinate of m
  int my = m1.getMeterY(); // y coordinate of m
  int mw = m1.getMeterWidth();
  
  m2 = new Meter(this, mx + mw + 20, my);
  
  m2.setTitleFontSize(20);
  m2.setTitleFontName("Monospaced");
  m2.setTitle("Ultrasonic Sensor (in)");
  
  // Change meter scale values
  String[] scaleLabels2 = {"0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "110", "120", "130"};
  m2.setScaleLabels(scaleLabels2);
  m2.setScaleFontSize(18);
  m2.setScaleFontName("Monospaced");
  m2.setScaleFontColor(color(200, 30, 70));
  
  // We can also display the value of meter
  m2.setDisplayDigitalMeterValue(true);
  
  // Lets do some more modifications so our meter looks nice
  m2.setArcColor(color(141, 113, 178));
  m2.setArcThickness(15);
  m2.setMaxScaleValue(130);
  m2.setMinInputSignal(0);
  m2.setMaxInputSignal(128);
  m2.setNeedleThickness(3);
}

void draw()
{
  // Lets give title to our window
  background(0);
  textSize(30);
  fill(0, 0, 255); // Font color , (r,g,b)
  text("Sensors & Motors Lab", 500, 40); // ("text", x, y)
  
  m1.updateMeter(force); 
  m2.updateMeter(ultra);
   
  textSize(20);
  fill(23, 253, 69);
  text("Slot Sensor", 50, 120);
  if (slot == 1) fill(255, 0, 0);
  else fill(0, 255, 0);
  ellipse(100, 200, 100, 100);
}

void serialEvent(Serial port) 
{
  // read a byte from the serial port:
  int inByte = port.read();
  // if this is the first byte received, and it's an A,
  // clear the serial buffer and note that you've
  // had first contact from the microcontroller.
  // Otherwise, add the incoming byte to the array:
  if (firstContact == false && inByte == 'A') 
  {
      port.clear();          // clear the serial port buffer
      firstContact = true;     // you've had first contact from the microcontroller
      port.write('A');       // ask for more
  }
  else 
  {
    // Add the latest byte from the serial port to array:
    serialInArray[serialCount] = inByte;
    serialCount++;

    // If we have 4 bytes:
    if (serialCount > 3 ) 
    {
      slot = serialInArray[0];
      force = (serialInArray[1] << 8) | serialInArray[2];
      ultra = serialInArray[3];

      // print the values (for debugging purposes only):
      println(slot + "\t" + ultra + "\t" + force);

      // Send motor values:
      for (int i=0; i<7; i++) port.write(serialOutArray[i]);
      
      // Reset serialCount:
      serialCount = 0;
    }
  }
}

void controlEvent(ControlEvent theEvent) 
{
  if(theEvent.isController()) 
  { 
    print("control event from : "+theEvent.getController().getName());
    println(", value : "+theEvent.getController().getValue());
   
    // changing the value of serialOutArray
    if(theEvent.getController().getName()=="ServoPos") {
      serialOutArray[0] = int(theEvent.getController().getValue());
    }
    else if(theEvent.getController().getName()=="DCPos") {
      int tmp = int(theEvent.getController().getValue());
      serialOutArray[1] = tmp / 256;
      serialOutArray[2] = tmp % 256;
    }
    else if(theEvent.getController().getName()=="DCVel") {
      serialOutArray[3] = int(theEvent.getController().getValue());
    }
    else if(theEvent.getController().getName()=="StepperPos") {
       int tmp = int(theEvent.getController().getValue());
       serialOutArray[4] = tmp / 256;
       serialOutArray[5] = tmp % 256;
    }
    else if(theEvent.getController().getName()=="GUI-Sensor") {
      gui = !gui;
      serialOutArray[6] = int(gui);
    }
   }
 } 
