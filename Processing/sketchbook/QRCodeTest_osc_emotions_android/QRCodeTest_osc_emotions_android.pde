import ketai.camera.*;
import ketai.cv.facedetector.*;
import ketai.data.*;
import ketai.net.*;
import ketai.net.bluetooth.*;
import ketai.net.nfc.*;
import ketai.net.nfc.record.*;
import ketai.net.wifidirect.*;
import ketai.sensors.*;
import ketai.ui.*;

import netP5.*;
import oscP5.*;

OscP5 oscP5;

NetAddress receiver;

//import processing.vr.*;

import processing.video.*;
import com.google.zxing.*;
import java.io.ByteArrayInputStream;
import javax.imageio.ImageIO;
import com.google.zxing.common.*;
import com.google.zxing.client.j2se.*;
import processing.opengl.*;

import java.awt.image.BufferedImage;

//import processing.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.google.zxing.*;
import java.io.ByteArrayInputStream;
import javax.imageio.ImageIO;
import com.google.zxing.common.*;
import com.google.zxing.client.j2se.*;
import android.graphics.Bitmap;
import ketai.camera.*;

//Client c;
//String input;
byte data[];

int pointspercolor=0x15;

int[][] colors = new int[pointspercolor*5][3];
float[][] points = new float[pointspercolor*5][2];
int[] pointscolors = new int[pointspercolor*5];

//String path="/home/dmitryneuro/sketchbook/QRCodeTest_telnet/";
String path="";
PImage[][] img = new PImage[3][3];

byte col;

KetaiCamera cam;

PImage camimg;

//Capture cam; //Set up the camera
com.google.zxing.Reader reader = new com.google.zxing.MultiFormatReader();
boolean globalHistogram = false;
void setup() {
  
    oscP5 = new OscP5( this , 12000 );
  receiver = new NetAddress( "192.168.1.71" , 12000 );

  //fullScreen(STEREO);
  //orientation(LANDSCAPE);
  //size(640, 480);
  path="";
img[0][0] = loadImage(path+"11.png");
img[0][1] = loadImage(path+"12.png");
img[0][2] = loadImage(path+"13.png");
img[1][0] = loadImage(path+"21.png");
img[1][1] = loadImage(path+"22.png");
img[1][2] = loadImage(path+"23.png");
img[2][0] = loadImage(path+"31.png");
img[2][1] = loadImage(path+"32.png");
img[2][2] = loadImage(path+"33.png");
                   
  //size(640, 480,P3D);
  //  frameRate(16);
 //surface.setLocation(100, 100);
  //size(640, 480,P2D);
  //size(320, 240);
  
  //fullScreen();
  size(854, 480);

  orientation(LANDSCAPE);
  //orientation(PORTRAIT);
  //imageMode(CENTER);
  //cam = new KetaiCamera(this, 320, 240, 24);
  //cam = new KetaiCamera(this, 640, 480, 24);
  cam = new KetaiCamera(this, 864, 480, 24);
  //cam = new KetaiCamera(this, 1920, 1080, 24);
  
  //String[] cameras = Capture.list();
//if (cameras.length == 0) {
//    println("There are no cameras available for capture.");
//    exit();
//  } else {
//    println("Available cameras:");
//    for (int i = 0; i < cameras.length; i++) {
//      println(cameras[i]);
//    }
//  //cam = new Capture(this,cameras[0]);
//  //cam = new Capture(this, 640, 480,"/dev/video1");
//   //cam = new Capture(this, 640, 480,"/dev/video0");
//   cam = new Capture(this, Capture.list()[0]);
   
 
//  //cam = new Capture(this,cameras[cameras.length-1]);
//  //cam = new Capture(this, 320, 240);
//  //size(cam.width, cam.height);
//  cam.start();
//  }
  
  cam.start();
  
  //c = new Client(this, "127.0.0.1", 1337); // Replace with your server's IP and port
  //c = new Client(this, "100.90.19.10", 1337); // Replace with your server's IP and port
  //int readdata=0;
  //while (readdata<32)
  //{
    //if (c.available() > 32-1) {
      //data = c.readBytes(32);
      //readdata+=32;
    //}
  //}
  
  for(int i1=0;i1<pointspercolor;i1++)
  {
    colors[pointspercolor*0+i1][0] = 0xff;
    colors[pointspercolor*0+i1][1] = (i1)*0x100/pointspercolor;
    colors[pointspercolor*0+i1][2] = 0;
  }
  for(int i1=0;i1<pointspercolor;i1++)
  {
    colors[pointspercolor*1+i1][0] = (pointspercolor-i1-1)*0x100/pointspercolor;
    colors[pointspercolor*1+i1][1] = 0xff;
    colors[pointspercolor*1+i1][2] = 0;
  }
  for(int i1=0;i1<pointspercolor;i1++)
  {
    colors[pointspercolor*2+i1][0] = 0;
    colors[pointspercolor*2+i1][1] = 0xff;
    colors[pointspercolor*2+i1][2] = (i1)*0x100/pointspercolor;
  }
  for(int i1=0;i1<pointspercolor;i1++)
  {
    colors[pointspercolor*3+i1][0] = 0;
    colors[pointspercolor*3+i1][1] = (pointspercolor-i1-1)*0x100/pointspercolor;
    colors[pointspercolor*3+i1][2] = 0xff;
  }
  for(int i1=0;i1<pointspercolor;i1++)
  {
    colors[pointspercolor*4+i1][0] = 0;
    colors[pointspercolor*4+i1][1] = 0;
    colors[pointspercolor*4+i1][2] = (pointspercolor-i1-1)*0x100/pointspercolor;
  }
  for(int i1=0;i1<pointspercolor*5;i1++)
  {
    points[i1][0] = 0;
    points[i1][1] = 0;
    pointscolors[i1] = pointspercolor*5-1;
  }
 col=0;

}

double d0 = 0;  
double d1 = 0;  

void oscEvent( OscMessage m ) 
{
  if(m.checkAddrPattern("/valence")==true) 
  {
    if(m.checkTypetag("f")) 
    {
      d0 = m.get(0).floatValue();
    }
  }
  if(m.checkAddrPattern("/arousal")==true) 
  {
    if(m.checkTypetag("f")) 
    {
      d1 = m.get(0).floatValue();
    }
  }
}

        float posX0 = 0;
        float posY0 = 0;
        float posX1 = 0;
        float posY1 = 0;
        float posX2 = 0;
        float posY2 = 0;


void draw() {
  
//byte[] bytes0 = { };
  //int readdata0=0;
  //while (readdata0<8)
  //{
    //if (c.available() > 8-1) {
      //bytes0 = c.readBytes(8);
      //readdata0+=8;
    //}
  //}
  
//double d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();  

//byte[] bytes1 = { };
//  int readdata1=0;
//  while (readdata1<8)
//  {
//    if (c.available() > 8-1) {
//      bytes1 = c.readBytes(8);
//      readdata1+=8;
//    }
//  }
  
//double d1 = ByteBuffer.wrap(bytes1).order(ByteOrder.LITTLE_ENDIAN).getDouble();  

//double d0 = 0;

  float f0=(float)d0;
  float f1=(float)d1;
  
  for(int i1=0;i1<pointspercolor*5;i1++)
  {
    pointscolors[i1]++;
    if(pointscolors[i1]>pointspercolor*5-1)
    {
      pointscolors[i1]=pointspercolor*5-1;
    }
  }
    points[col][0] = f0;
    points[col][1] = f1;
    pointscolors[col] = 0;
    col++;
    if(col>pointspercolor*5-1)
    {
      col=0;
    }
  
   
  ////if (cam.available() == true) 
  //if (cam != null && cam.isStarted())
  //{
  //  //cam.read();
  //  tint(255, 255);
    //image(cam, 0, 0);
    if(cam != null) 
    {
      image(cam, 0, 0);
      
  Bitmap camBitmap = (Bitmap) cam.getNative();
  //camimg = (PImage)camBitmap;
  //Object cambuf = cam.getNative();
  //camimg = new PImage(cambuf);
  //Bitmap camBitmap = (Bitmap) cambuf;
  int w = camBitmap.getWidth();
  int h = camBitmap.getHeight();
  int[] rgb = new int[w * h];
  byte[] yuv = new byte[w * h];

  camBitmap.getPixels(rgb, 0, w, 0, 0, w, h);
  populateYUVLuminanceFromRGB(rgb, yuv, w, h);
  PlanarYUVLuminanceSource source = new PlanarYUVLuminanceSource(yuv, w, h, 0, 0, w, h, false);
  BinaryBitmap bitmap;
  if (globalHistogram)
    bitmap = new BinaryBitmap(new GlobalHistogramBinarizer(source)); 
  else

    bitmap = new BinaryBitmap(new HybridBinarizer(source)); 
  Result result = null;
  try {
    result = reader.decode(bitmap);
  } 
  catch (Exception e) {
  }
  //Once we get the results, we can do some display
  if (result != null &&
    result.getText() != null) { 
        //println(result.getText());
        //println(result.getResultPoints().toString());
         posX0 = result.getResultPoints()[0].getX();
         posY0 = result.getResultPoints()[0].getY();
         posX1 = result.getResultPoints()[1].getX();
         posY1 = result.getResultPoints()[1].getY();
         posX2 = result.getResultPoints()[2].getX();
         posY2 = result.getResultPoints()[2].getY();
  }
      
    }
    
    

  //  BufferedImage buf = (BufferedImage) cam.getNative();
  //  // Now test to see if it has a QR code embedded in it
  //  LuminanceSource source = new BufferedImageLuminanceSource(buf);
  //  BinaryBitmap bitmap;
  //  if (globalHistogram)
  //    bitmap = new BinaryBitmap(new GlobalHistogramBinarizer(source)); 
  //  else

  //      bitmap = new BinaryBitmap(new HybridBinarizer(source)); 
  //  Result result = null;
  //  try {
  //    result = reader.decode(bitmap);
  //  } 
  //  catch (Exception e) {
      
  //  }
  //  //Once we get the results, we can do some display
  //  if (result != null &&
  //    result.getText() != null) { 
  //  }
  //}
  
        stroke(255,0,0);
        float posX00 = posX1-(posX1-posX0)/2;
        float posY00 = posY1-(posY1-posY0)/2;
        float posX01 = posX2-(posX1-posX0)/2;
        float posY01 = posY2-(posY1-posY0)/2;
        line(posX00,posY00,posX01,posY01);
        //line(posX0,posY0,posX1,posY1);
        stroke(0,255,0);
        float posX02 = posX1-(posX1-posX2)/2;
        float posY02 = posY1-(posY1-posY2)/2;
        float posX03 = posX0-(posX1-posX2)/2;
        float posY03 = posY0-(posY1-posY2)/2;
        line(posX02,posY02,posX03,posY03);
        //line(posX1,posY1,posX2,posY2);
        //for(int i1=0;i1<0xf;i1++)
        //{
        //stroke((0xf-i1)*0xf, i1*0xf, 0);
        //  fill((0xf-i1)*0xf, i1*0xf, 0);
  for(int i11=0;i11<pointspercolor*5;i11++)
  {
    int i1=i11+col;
    if (i1>pointspercolor*5-1) 
    {
      i1=i1-(pointspercolor*5);
    }
    stroke(colors[pointscolors[i1]][0],
           colors[pointscolors[i1]][1],
           colors[pointscolors[i1]][2]);
    fill(colors[pointscolors[i1]][0],
         colors[pointscolors[i1]][1],
         colors[pointscolors[i1]][2]);
          float posX000 = points[i1][0];
          float posY000 = points[i1][1];
          float len3 = sqrt(sq(posX01-posX00)+sq(posY03-posY02));
          float lenX3 = (posX01-posX00);
          float lenY3 = (posY03-posY02);
          float posX0000 = posX00+lenX3/2;
          float posY0000 = posY02+lenY3/2;
          //float posX0001 = posX0000-posX00;
          //float posY0001 = posY0000-posY02;
          float posX00000 = posX0000+posX000*abs(lenX3/2);
          float posY00000 = posY0000+posY000*abs(lenY3/2);
          float lenX4 = (((len3)/(25-7))/(1));
          float lenY4 = (((len3)/(25-7))/(1));
          //rect(posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          if((posX000<-1+2/3)&&(posY000<-1+2/3))
          {
            image(img[0][2], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posX000<-1+2/3)&&(posY000<-1+4/3))
          {
            image(img[0][1], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posX000<-1+2/3))
          {
            image(img[0][0], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posX000<-1+4/3)&&(posY000<-1+2/3))
          {
            image(img[1][2], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posX000<-1+4/3)&&(posY000<-1+4/3))
          {
            image(img[1][1], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posX000<-1+4/3))
          {
            image(img[1][0], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posY000<-1+2/3))
          {
            image(img[2][2], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          if((posY000<-1+4/3))
          {
            image(img[2][1], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          } else
          {
            image(img[2][0], posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
          }
       // }
       // for(int i1=0;i1<0xf;i1++)
       // {
       //   stroke(0,(0xf-i1)*0xf,i1*0xf);
       //   fill(0,(0xf-i1)*0xf,i1*0xf);
       //   float posX000 = random(-1,1);
       //   float posY000 = random(-1,1);
       //   float len3 = sqrt(sq(posX01-posX00)+sq(posY03-posY02));
       //   float lenX3 = posX01-posX00;
       //   float lenY3 = posY03-posY02;
       //   float posX0000 = posX00+lenX3/2;
       //   float posY0000 = posY02+lenY3/2;
       //   //float posX0001 = posX0000-posX00;
       //   //float posY0001 = posY0000-posY02;
       //   float posX00000 = posX0000+posX000*abs(lenX3/2);
       //   float posY00000 = posY0000+posY000*abs(lenY3/2);
       //   float lenX4 = (((len3)/(25-7))/(2*2*2));
       //   float lenY4 = (((len3)/(25-7))/(2*2*2));
       //   rect(posX00000-lenX4, posY00000-lenY4, lenX4*2, lenY4*2);
       //}
        }  
}

void populateYUVLuminanceFromRGB(int[] rgb, byte[] yuv420sp, int width, int height) {
  for (int i = 0; i < width * height; i++) {
    float red = (rgb[i] >> 16) & 0xff;
    float green = (rgb[i] >> 8) & 0xff;
    float blue = (rgb[i]) & 0xff;
    int luminance = (int) ((0.257f * red) + (0.504f * green) + (0.098f * blue) + 16);
    yuv420sp[i] = (byte) (0xff & luminance);
  }
}
void onCameraPreviewEvent()
{
  cam.read();
  
}

void mousePressed()
{
      if (cam.isFlashEnabled())
        cam.disableFlash();
      else
        cam.enableFlash();
}
