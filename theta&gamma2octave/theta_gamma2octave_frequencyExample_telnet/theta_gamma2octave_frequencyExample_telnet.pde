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

// create all of the variables that will need to be accessed in
// more than one methods (setup(), draw(), stop()).
Minim minim;
AudioOutput out;

int tg=10;
Oscil[]      wave = new Oscil[tg];
// keep track of the current Frequency so we can display it
Frequency[]  currentFreq = new Frequency[tg];

import processing.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

Client c;
String input;
int maxdata=256;
double[] data = new double[maxdata];
int dataread0=0;

// setup is run once at the beginning
void setup()
{
  
      c = new Client(this, "127.0.0.1", 1337); // Replace with your server's IP and port

  // initialize the drawing window
  size(512, 200);
  
  // initialize the minim and out objects
  minim = new Minim(this);
  out   = minim.getLineOut();

  for(int i=0;i<tg;i++)
  {
    currentFreq[i] = Frequency.ofPitch( "A4" );
    //wave[i] = new Oscil( currentFreq[i], 0.6f, Waves.TRIANGLE );
    wave[i] = new Oscil( currentFreq[i], 0.6f, Waves.SINE );
    wave[i].patch( out );
  }

  
  
}

// draw is run many times
void draw()
{
  
      dataread0=0;
  double d0=0;
  double d1=0;
  int data0=0;
  
  while(d0!=-1)
  {
    byte[] bytes0 = { };
    int readdata0=0;
    while (readdata0<8)
    {
      if (c.available() > 8-1) {
        bytes0 = c.readBytes(8);
        readdata0+=8;
      }
    }
    d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
  }
  while(d0==-1)
  {
    byte[] bytes0 = { };
    int readdata0=0;
    while (readdata0<8)
    {
      if (c.available() > 8-1) {
        bytes0 = c.readBytes(8);
        readdata0+=8;
      }
    }
    d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
  }

  int tg1=0;
  while(d0!=-1)
  {
    byte[] bytes0 = { };
    int readdata0=0;
    while (readdata0<8)
    {
      if (c.available() > 8-1) {
        bytes0 = c.readBytes(8);
        readdata0+=8;
      }
    }
    d0 = ByteBuffer.wrap(bytes0).order(ByteOrder.LITTLE_ENDIAN).getDouble();
    if(d0!=-1)
    {
      data[dataread0]=d0;
      //table.set(dataread0,d0);
      //table.getWaveform()[440+dataread0]=(float)d0/2;
      if(d0>d1)
      {
        data0=dataread0;
        d1=d0;
      }
      if((d0==0)&&(d1>0)&&(tg1<tg))
      {
        float modulateAmount = (float)d1*0.1;
        float modulateFrequency = 329.6+data0*4;
        wave[tg1].setFrequency( modulateFrequency );
        wave[tg1].setAmplitude( modulateAmount );
        tg1++;
        d1=0;
      }
      dataread0++;
    }
  }
  for(int i = tg1; i < tg; i++)
  {
        float modulateAmount = (float)0;
        float modulateFrequency = 0;
        wave[i].setFrequency( modulateFrequency );
        wave[i].setAmplitude( modulateAmount );
  }
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
  background( 64, 32, 0 );
  // draw using a beige stroke
  stroke( 255, 238, 192 );
  
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
}
