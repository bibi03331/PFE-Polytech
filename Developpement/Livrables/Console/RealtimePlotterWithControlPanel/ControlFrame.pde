// Settings for the control panel are saved in this file
JSONObject robotConfigJSON;


ControlFrame addControlFrame(String theName, int theWidth, int theHeight) {
  Frame f = new Frame(theName);
  ControlFrame p = new ControlFrame(this, theWidth, theHeight);
  f.add(p);
  p.init();
  f.setTitle(theName);
  f.setSize(p.w, p.h);
  f.setLocation(100, 100);
  f.setResizable(false);
  f.setVisible(true);
  return p;
}

// the ControlFrame class extends PApplet, so we 
// are creating a new processing applet inside a
// new frame with a controlP5 object loaded
public class ControlFrame extends PApplet {

  int w, h;

  int abc = 100;

  public void setup() {
    size(w, h);
    frameRate(25);
    cp5 = new ControlP5(this);

    robotConfigJSON = loadJSONObject(topSketchPath+"/robot_config.json");
    
    // PID
    int x;
    int y;
    
    // Pitch
    cp5.addTextlabel("label1").setText("PID axe PITCH").setPosition(x=5, y=5).setFont(createFont("Georgia", 14));
    cp5.addTextfield("Kp_Pitch").setPosition(x=x+5, y=y+20).setText(getConfigString("Kp_Pitch")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Ki_Pitch").setPosition(x, y=y+40).setText(getConfigString("Ki_Pitch")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Kd_Pitch").setPosition(x, y=y+40).setText(getConfigString("Kd_Pitch")).setWidth(40).setAutoClear(false);

    y=y-100;
    x=x+150;

    // Roll
    cp5.addTextlabel("label2").setText("PID axe ROLL").setPosition(x, y).setFont(createFont("Georgia", 14));
    cp5.addTextfield("Kp_Roll").setPosition(x=x+5, y=y+20).setText(getConfigString("Kp_Roll")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Ki_Roll").setPosition(x, y=y+40).setText(getConfigString("Ki_Roll")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Kd_Roll").setPosition(x, y=y+40).setText(getConfigString("Kd_Roll")).setWidth(40).setAutoClear(false);
    
    y=y-100;
    x=x+150;

    // Yaw
    cp5.addTextlabel("label3").setText("PID axe YAW").setPosition(x, y).setFont(createFont("Georgia", 14));
    cp5.addTextfield("Kp_Yaw").setPosition(x=x+5, y=y+20).setText(getConfigString("Kp_Yaw")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Ki_Yaw").setPosition(x, y=y+40).setText(getConfigString("Ki_Yaw")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("Kd_Yaw").setPosition(x, y=y+40).setText(getConfigString("Kd_Yaw")).setWidth(40).setAutoClear(false);

    y=y+60;
    x=x-315;

    // Enable / disable axis
    cp5.addTextlabel("label4").setText("Activation / desactivation des axes").setPosition(x, y).setFont(createFont("Georgia", 14));
    cp5.addToggle("en_yaw_axis").setPosition(x=x+5, y=y+20).setValue(int(getConfigString("en_yaw_axis"))).setMode(ControlP5.SWITCH);
    cp5.addToggle("en_pitch_axis").setPosition(x, y=y+40).setValue(int(getConfigString("en_pitch_axis"))).setMode(ControlP5.SWITCH);
    cp5.addToggle("en_roll_axis").setPosition(x, y=y+40).setValue(int(getConfigString("en_roll_axis"))).setMode(ControlP5.SWITCH);
    
    y=y-100;
    x=x+250;
    
    // Frequences
    cp5.addTextlabel("label5").setText("Frequences des interruptions").setPosition(x, y).setFont(createFont("Georgia", 14));
    cp5.addTextfield("freq_cpt").setPosition(x=x+5, y=y+20).setText(getConfigString("freq_cpt")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("freq_motors").setPosition(x, y=y+40).setText(getConfigString("freq_motors")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("freq_PID").setPosition(x, y=y+40).setText(getConfigString("freq_PID")).setWidth(40).setAutoClear(false);
    cp5.addTextfield("freq_madgwick").setPosition(x, y=y+40).setText(getConfigString("freq_madgwick")).setWidth(40).setAutoClear(false);

    PImage[] imgs = {
      loadImage(topSketchPath+"/images/button_a.png"), loadImage(topSketchPath+"/images/button_b.png"), loadImage(topSketchPath+"/images/button_c.png")
    };
    
    x = 200;
    cp5.addButton("moveForwards").setValue(1).setPosition(x, y=y+60).setImages(imgs).updateSize();
    cp5.addButton("moveBackwards").setValue(1).setPosition(x, y=y+60).setImages(imgs).updateSize();
    cp5.addButton("turnLeft").setValue(1).setPosition(x=x-60, y).setImages(imgs).updateSize();
    cp5.addButton("turnRight").setValue(1).setPosition(x=x+120, y).setImages(imgs).updateSize();
    
    cp5.addButton("start").setValue(1).setPosition(x=x-250, y=y-60);
    cp5.addButton("record_parameters").setValue(1).setPosition(x, y=y+40);
    cp5.addButton("stop").setValue(1).setPosition(x, y=y+40);
    cp5.addButton("calibrate").setValue(1).setPosition(x, y=y+40);
  }

  void controlEvent(ControlEvent theEvent) {
    print(theEvent);
    if (theEvent.isAssignableFrom(Textfield.class) || theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class)) {
      String parameter = theEvent.getName();
      String value = "";
      if (theEvent.isAssignableFrom(Textfield.class))
        value = theEvent.getStringValue();
      else if (theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class))
        value = theEvent.getValue()+"";

      robotConfigJSON.setString(parameter, value);
      saveJSONObject(robotConfigJSON, topSketchPath+"/robot_config.json");
      if (!mockupSerial) {
        serialPort.write(parameter+" "+value+";");
        serialPort.clear();
      }
      print(parameter+" "+value+";\n");
      /*for (int i=0; i<inBuffer.length; i++) {
       inBuffer[i] = 0;  
       }*/
    }
  }

  public void draw() {
    background(abc);
  }

  private ControlFrame() {
  }

  public ControlFrame(Object theParent, int theWidth, int theHeight) {
    parent = theParent;
    w = theWidth;
    h = theHeight;
  }


  public ControlP5 control() {
    return cp5;
  }


  ControlP5 cp5;

  Object parent;
}
String getConfigString(String id) {
  String r = "";
  try {
    r = robotConfigJSON.getString(id);
  } 
  catch (Exception e) {
    r = "";
  }
  return r;
}

