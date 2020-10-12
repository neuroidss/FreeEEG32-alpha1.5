/* frequencyExample<br/>
   is an example of using the Frequency class to easily turn keyboard input 
   into the frequency of an Oscil. Simply type on the home row to change 
   the pitch of the tone. 
   <p>
   For more information about Minim and additional features, 
   visit http://code.compartmental.net/minim/
*/

// import everything necessary to make sound.
import ddf.minim.*;
import ddf.minim.ugens.*;
import ddf.minim.signals.*;


// create all of the variables that will need to be accessed in
// more than one methods (setup(), draw(), stop()).
Minim minim;
AudioOutput out;
AudioRecorder recorder;

  //Pan[] pans;
  //float[] pans = new float[2];
  Pan[] pans = new Pan[2];




import netP5.*;
import oscP5.*;


import java.io.ByteArrayInputStream;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import java.io.ByteArrayInputStream;

byte data[];


int tg=10;
int ctg1=2;
int ctg2=4;
int ctg10=0;
int ctg20=0;
int ctg11=2;
int ctg21=4;
int addr; 
//int[][] addr = new int[ctg1][ctg2]; 
String[][] addrpatterns = new String[ctg1][ctg2];

OscP5 oscP5;
//OscP5[][] oscP5= new OscP5[ctg1][ctg2];

//NetAddress[][] receiver[ctg1][ctg2];

//Oscil[]      wave = new Oscil[tg];
Oscil[][][]      wave = new Oscil[ctg1][ctg2][tg];
// keep track of the current Frequency so we can display it
//Frequency[]  currentFreq = new Frequency[tg];
Frequency[][][]  currentFreq = new Frequency[ctg1][ctg2][tg];

//SineWave[][][]      sine = new SineWave[ctg1][ctg2][tg];

import processing.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

//Client c;
String input;
//int maxdata=256;
//double[] data = new double[maxdata];
//int dataread0=0;

float[][] amounts = new float[ctg1][ctg2];
float[][] frequencies = new float[ctg1][ctg2];
float[][] frequenciesmulti = new float[ctg1][ctg2];

float[][][] ds = new float[ctg1][ctg2][4];  
//double d0 = 0;  
//double d1 = 0;  
int[][] tgs1 = new int[ctg1][ctg2];

//Gain gain;

float base=pow(2,4);
float amountbase=1;
float frequencybase=110;
float frequenciemultibase=4;

// setup is run once at the beginning
void setup()
{

  for(int i1=0; i1<ctg1; i1++)
  {
    for(int i2=0; i2<ctg2; i2++)
    {
      amounts[i1][i2] = amountbase/(base*(pow(2,ctg2-i2)));
      frequencies[i1][i2] = frequencybase*base/(pow(2,i2));
      frequenciesmulti[i1][i2] = frequenciemultibase*base/(pow(2,i2));
    }
  }
  
  //amounts[0][0] = 0.01;
  //amounts[0][1] = 0.01;
  //amounts[0][2] = 0.01;
  //amounts[0][3] = 0.01;
  //amounts[1][0] = 0.01;
  //amounts[1][1] = 0.01;
  //amounts[1][2] = 0.01;
  //amounts[1][3] = 0.01;
  //frequencies[0][0] = 880;
  //frequencies[0][1] = 440;
  //frequencies[0][2] = 220;
  //frequencies[0][3] = 110;
  //frequencies[1][0] = 880;
  //frequencies[1][1] = 440;
  //frequencies[1][2] = 220;
  //frequencies[1][3] = 110;
  ////frequencies[0][0] = 440;
  ////frequencies[0][1] = 220;
  ////frequencies[0][2] = 110;
  ////frequencies[0][3] = 55;
  ////frequencies[1][0] = 440;
  ////frequencies[1][1] = 220;
  ////frequencies[1][2] = 110;
  ////frequencies[1][3] = 55;
  //frequenciesmulti[0][0] = 8;
  //frequenciesmulti[0][1] = 4;
  //frequenciesmulti[0][2] = 2;
  //frequenciesmulti[0][3] = 1;
  //frequenciesmulti[1][0] = 8;
  //frequenciesmulti[1][1] = 4;
  //frequenciesmulti[1][2] = 2;
  //frequenciesmulti[1][3] = 1;

  addrpatterns[0][0] = "/theta-gamma-Fp1"; 
  addrpatterns[0][1] = "/theta-gamma-F3"; 
  addrpatterns[0][2] = "/theta-gamma-P3"; 
  addrpatterns[0][3] = "/theta-gamma-O1"; 
  addrpatterns[1][0] = "/theta-gamma-Fp2"; 
  addrpatterns[1][1] = "/theta-gamma-F4"; 
  addrpatterns[1][2] = "/theta-gamma-P4"; 
  addrpatterns[1][3] = "/theta-gamma-O2";
  
  addr = 9001; 
  //addr[0][0] = 9001; 
  //addr[0][1] = 9101; 
  //addr[0][2] = 9201; 
  //addr[0][3] = 9301; 
  //addr[1][0] = 9011; 
  //addr[1][1] = 9111; 
  //addr[1][2] = 9211; 
  //addr[1][3] = 9311;
  
  for(int i1=0; i1<ctg1; i1++)
  {
    for(int i2=0; i2<ctg2; i2++)
    {
      tgs1[i1][i2]=0;
      for(int i3=0; i3<4; i3++)
      {
        ds[i1][i2][i3]=0;
      }
    }
  }
  
  
  //receiver = new NetAddress( "192.168.1.71" , 12000 );
  
  
      //c = new Client(this, "127.0.0.1", 1337); // Replace with your server's IP and port

  // initialize the drawing window
  size(512, 200);
  
  // initialize the minim and out objects
  minim = new Minim(this);
  out = minim.getLineOut(Minim.STEREO);
  //out   = minim.getLineOut();

  Summer[] synth = new Summer[2];
  synth[0] = new Summer();
  synth[1] = new Summer();
  
  //pans[0] = new Pan(0);
  //pans[1] = new Pan(0);
  pans[0] = new Pan(-1);
  pans[1] = new Pan(1);
  //pans[0] = -1;
  //pans[1] = 1;
  
  //gain = new Gain(0);
  
  

  //for(int i1=0; i1<ctg1; i1++)
  //{
  //  for(int i2=0; i2<ctg2; i2++)
  //  {
  for(int i1=ctg10; i1<ctg11; i1++)
  {
    for(int i2=ctg20; i2<ctg21; i2++)
    {
      for(int i=0;i<tg;i++)
      {
        currentFreq[i1][i2][i] = Frequency.ofPitch( "A4" );
        //wave[i] = new Oscil( currentFreq[i], 0.6f, Waves.TRIANGLE );
        wave[i1][i2][i] = new Oscil( currentFreq[i1][i2][i], 0, Waves.SINE );
        //wave[i1][i2][i] = new Oscil( currentFreq[i1][i2][i], 0.6f, Waves.SINE );
        //wave[i1][i2][i].patch( out );
        //wave[i1][i2][i].patch( pans[i1] );
        wave[i1][i2][i].patch( synth[i1] );

        //sine[i1][i2][i] = new SineWave(frequencies[i1][i2], amounts[i1][i2], out.sampleRate());
        //sine[i1][i2][i].portamento(200);
        //out.addSignal(sine[i1][i2][i]);

      }
    }
  }

  synth[0].patch( pans[0] );
  synth[1].patch( pans[1] );

  pans[0].patch( out );
  pans[1].patch( out );


  recorder = minim.createRecorder(out, "myrecording.wav");
  
  //for(int i1=0; i1<ctg1; i1++)
  //{
  //  for(int i2=0; i2<ctg2; i2++)
  //  {
      oscP5 = new OscP5( this , addr );
      //oscP5[i1][i2] = new OscP5( this , addr[i1][i2] );
      //receiver[i1][i2] = new NetAddress( "127.0.0.1" , addr[i1][i2] );
  //  }
  //}
  
  
}



void oscEvent( OscMessage m ) 
{
  
  //for(int i1=0; i1<ctg1; i1++)
  //{
  //  for(int i2=0; i2<ctg2; i2++)
  //  {
  for(int i1=ctg10; i1<ctg11; i1++)
  {
    for(int i2=ctg20; i2<ctg21; i2++)
    {
      //tgs1[i1][i2]=0;
      for(int i3=0; i3<4; i3++)
      {
        //ds[i1][i2][i3]=0;
      }
      if(m.checkAddrPattern(addrpatterns[i1][i2])==true) 
      {
        if(m.checkTypetag("f")) 
        {
          ds[i1][i2][0] = m.get(0).floatValue();
          if(ds[i1][i2][0]>=0)
          {
      //data[dataread0]=d0;
      //table.set(dataread0,d0);
      //table.getWaveform()[440+dataread0]=(float)d0/2;
            if(ds[i1][i2][0]>ds[i1][i2][1])
            {
              ds[i1][i2][2]=ds[i1][i2][3];
              ds[i1][i2][1]=ds[i1][i2][0];
            }
            if((ds[i1][i2][0]==0)&&(ds[i1][i2][1]>0)&&(tgs1[i1][i2]<tg))
            {
              float modulateAmount = (float)ds[i1][i2][1]*amounts[i1][i2];
              float modulateFrequency = frequencies[i1][i2]+ds[i1][i2][2]*frequenciesmulti[i1][i2];
              wave[i1][i2][tgs1[i1][i2]].setFrequency( modulateFrequency );
              wave[i1][i2][tgs1[i1][i2]].setAmplitude( modulateAmount );
              tgs1[i1][i2]++;
              ds[i1][i2][1]=0;
            }
            ds[i1][i2][3]++;
          }
          else
          if(tgs1[i1][i2]!=0)
          {
            for(int i = tgs1[i1][i2]; i < tg; i++)
            {
              float modulateAmount = (float)0;
              float modulateFrequency = 0;
              //wave[i1][i2][i].setFrequency( modulateFrequency );
              wave[i1][i2][i].setAmplitude( modulateAmount );
            }
            tgs1[i1][i2]=0;
            for(int i3=0; i3<4; i3++)
            {
              ds[i1][i2][i3]=0;
            }
          }
        }
      }
    }
  }
  
      //dataread0=0;
  //int[][] data0=0;
  
}


// draw is run many times
void draw()
{
  
  //    dataread0=0;
  //double d0=0;
  //double d1=0;
  //int data0=0;
  
  //while(d0!=-1)
  //{
  //  byte[] bytes0 = { };
  //  int readdata0=0;
  //  while (readdata0<8)
  //  {
  //    if (c.available() > 8-1) {
  //      bytes0 = c.readBytes(8);
  //      readdata0+=8;
  //    }
  //  }
  //  d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
  //}
  //while(d0==-1)
  //{
  //  byte[] bytes0 = { };
  //  int readdata0=0;
  //  while (readdata0<8)
  //  {
  //    if (c.available() > 8-1) {
  //      bytes0 = c.readBytes(8);
  //      readdata0+=8;
  //    }
  //  }
  //  d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
  //}

  //int tg1=0;
  //while(d0!=-1)
  //{
  //  byte[] bytes0 = { };
  //  int readdata0=0;
  //  while (readdata0<8)
  //  {
  //    if (c.available() > 8-1) {
  //      bytes0 = c.readBytes(8);
  //      readdata0+=8;
  //    }
  //  }
  //  d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
  //  if(d0!=-1)
  //  {
  //    data[dataread0]=d0;
  //    //table.set(dataread0,d0);
  //    //table.getWaveform()[440+dataread0]=(float)d0/2;
  //    if(d0>d1)
  //    {
  //      data0=dataread0;
  //      d1=d0;
  //    }
  //    if((d0==0)&&(d1>0)&&(tg1<tg))
  //    {
  //      float modulateAmount = (float)d1*0.1;
  //      float modulateFrequency = 329.6+data0*4;
  //      wave[tg1].setFrequency( modulateFrequency );
  //      wave[tg1].setAmplitude( modulateAmount );
  //      tg1++;
  //      d1=0;
  //    }
  //    dataread0++;
  //  }
  //}
  //for(int i = tg1; i < tg; i++)
  //{
  //      float modulateAmount = (float)0;
  //      float modulateFrequency = 0;
  //      wave[i].setFrequency( modulateFrequency );
  //      wave[i].setAmplitude( modulateAmount );
  //}
  //for(int i = 0; i < dataread0; i++)
  //{
  //    print(data[i]);
  //    print(",");
  //}
  //print("\n");

  //float modulateAmount = (float)d1/2;
  //float modulateFrequency = 440+data0;
  ////float modulateAmount = (float)d0/2;
  ////float modulateFrequency = 440+dataread0;
  
  //wave.setFrequency( modulateFrequency );
  //wave.setAmplitude( modulateAmount );
//  wave.setAmplitude( currentAmount );
//  wave.setFrequency( currentFreq );

  // erase the window to brown
  //background( 64, 32, 0 );
  //// draw using a beige stroke
  //stroke( 255, 238, 192 );
  
  //text( "Current Frequency in Hertz: " + currentFreq.asHz(), 5, 15 );
  //text( "Current Frequency as MIDI note: " + currentFreq.asMidiNote(), 5, 30 );
  
  // draw the waveforms
  //for( int i = 0; i < out.bufferSize() - 1; i++ )
  //{
  //  // find the x position of each buffer value
  //  float x1  =  map( i, 0, out.bufferSize(), 0, width );
  //  float x2  =  map( i+1, 0, out.bufferSize(), 0, width );
  //  // draw a line from one buffer position to the next for both channels
  //  line( x1, 50 + out.left.get(i)*50, x2, 50 + out.left.get(i+1)*50);
  //  line( x1, 150 + out.right.get(i)*50, x2, 150 + out.right.get(i+1)*50);
  //}  
}

// change the midi note when pressing keys on the keyboard
// we set midiNoteIn directly with the setMidiNoteIn method
// but you could also use a Line to lerp to the next note
// by patching it to midiNoteIn.
void keyPressed()
{
  if ( (key >= '1') && (key <= '9') )
  {
    base=pow(2,key-'1');
    for(int i1=0; i1<ctg1; i1++)
    {
      for(int i2=0; i2<ctg2; i2++)
      {
        amounts[i1][i2] = amountbase/(base*(pow(2,ctg2-i2)));
        frequencies[i1][i2] = frequencybase*base/(pow(2,i2));
        frequenciesmulti[i1][i2] = frequenciemultibase*base/(pow(2,i2));
      }
    }
  }
  //if ( key == 'a' ) currentFreq = Frequency.ofPitch( "A4" );
  //if ( key == 's' ) currentFreq = Frequency.ofPitch( "B4" );
  //if ( key == 'd' ) currentFreq = Frequency.ofPitch( "C#5" );
  //if ( key == 'f' ) currentFreq = Frequency.ofPitch( "D5" );
  //if ( key == 'g' ) currentFreq = Frequency.ofPitch( "E5" );
  //if ( key == 'h' ) currentFreq = Frequency.ofPitch( "F#5" );
  //if ( key == 'j' ) currentFreq = Frequency.ofPitch( "G#5" );
  //if ( key == 'k' ) currentFreq = Frequency.ofPitch( "A5" );
  //if ( key == 'l' ) currentFreq = Frequency.ofPitch( "B5" );
  //if ( key == ';' ) currentFreq = Frequency.ofPitch( "C#6" );
  //if ( key == '\'') currentFreq = Frequency.ofPitch( "E6" );
  
  // note that there are two other static methods for constructing Frequency objects
  // currentFreq = Frequency.ofHertz( 440 );
  // currentFreq = Frequency.ofMidiNote( 69 ); 
  
  //wave.setFrequency( currentFreq );

  if ( key == 'r' ) 
  {
    // to indicate that you want to start or stop capturing audio data, you must call
    // beginRecord() and endRecord() on the AudioRecorder object. You can start and stop
    // as many times as you like, the audio data will be appended to the end of the buffer 
    // (in the case of buffered recording) or to the end of the file (in the case of streamed recording). 
    if ( recorder.isRecording() ) 
    {
      recorder.endRecord();
    }
    else 
    {
      recorder.beginRecord();
    }
  }
  if ( key == 's' )
  {
    // we've filled the file out buffer, 
    // now write it to the file we specified in createRecorder
    // in the case of buffered recording, if the buffer is large, 
    // this will appear to freeze the sketch for sometime
    // in the case of streamed recording, 
    // it will not freeze as the data is already in the file and all that is being done
    // is closing the file.
    // the method returns the recorded audio as an AudioRecording, 
    // see the example  AudioRecorder >> RecordAndPlayback for more about that
    recorder.save();
    println("Done saving.");
  }

}

void stop()
{
  out.close();
  minim.stop();
  
  super.stop();
}
