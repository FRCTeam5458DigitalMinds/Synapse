#include "WPILib.h"
//#include <stdio>

#define kp							0.00001

//Default address of Pixy Camera. You can change the address of the Pixy in Pixymon under setting-> Interface
#define PIXY_I2C_DEFAULT_ADDR           0x54

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L   //x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

// RC-servo values - not needed unless you want to use servo to face the goal instead of moving the whole robot
#define PIXY_RCS_MIN_POS            0L
#define PIXY_RCS_MAX_POS            1000L
#define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)

enum BlockType
{
   NORMAL_BLOCK, //normal color recognition
   CC_BLOCK     //color-code(chnage in angle) recognition
};

struct Block
{
  // print block structure!
  void print()
  {
    int i, j;
    char buf[128], sig[6], d;
   bool flag;
    if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
   {
      // convert signature number to an octal string
      for (i=12, j=0, flag=false; i>=0; i-=3) //assigns value to signature, x, y, width, height, and anlge
      {
        d = (signature>>i)&0x07;
        if (d>0 && !flag)
          flag = true;
        if (flag)
          sig[j++] = d + '0';
      }
      sig[j] = '\0';
      printf("CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", sig, signature, x, y, width, height, angle);
    }
   else // regular block.  Note, angle is always zero, so no need to print
      printf("sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, width, height); //prints out data to console instead of smartDashboard -> check on the side of the driver station, check +print and click view console
    //Serial.print(buf);
  }
  uint16_t signature; //Identification number for your object - you could set it in the pixymon
  uint16_t x; //0 - 320
  uint16_t y; //0 - 200
  uint16_t width;
  uint16_t height;
  uint16_t angle;
};

class VisionProcessor
{
public:

   int maxIndex, secMaxIndex, x1, x2, average, numblocks, e;
   bool error = false, skip = false, badBlock = false;
   float max, secMax;
   I2C* i2c; //Declare i2c

   BlockType blockType;// it is the enum on the top
   bool  skipStart;   //skips to check 0xaa55, which is byte that tells pixy it is start of new frame
   uint16_t blockCount; //How many signatured objects are there?
   Block blocks[100]; //array that stores blockCount array

   VisionProcessor(I2C *wire){
   i2c = wire;
   }

   bool getStart() //checks whether if it is start of the normal frame, CC frame, or the data is out of sync
      {
        uint16_t w, lastw;

        lastw = 0xffff;

        while(true)
        {
          w = getWord(); //This it the function right underneath
          if (w==0 && lastw==0)
         {
            //delayMicroseconds(10);
           return false;
         }
          else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
         {
            blockType = NORMAL_BLOCK;
            return true;
         }
          else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
         {
            blockType = CC_BLOCK;
            return true;
         }
         else if (w==PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
         {
           printf("Pixy: reorder");
           getByte(); // resync
         }
         lastw = w;
        }
      }

   uint16_t getWord() //Getting two Bytes from Pixy (The full information)
   {
      unsigned char buffer[2] = {0, 0};

      i2c->ReadOnly(2, buffer);
      // printf("%02X%02X\n", buffer[0], buffer[1]);
      return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
   }

   uint8_t getByte()//gets a byte
   {
      unsigned char buffer[1] = {0};

      i2c->ReadOnly(1, buffer);
      return buffer[0];
   }

   uint16_t getBlocks(uint16_t maxBlocks)
   {
     blocks[0] = {0}; //resets the array - clears out data from previous reading
     uint8_t i;
     uint16_t w, checksum, sum;
     Block *block;

     if (!skipStart) //when computer has not seen 0xaa55 (starting frame)
     {
       if (getStart()==false)
         return 0;
     }
     else
      skipStart = false;

     for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
     {
       checksum = getWord();
       if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
       {
         skipStart = true; //starts this function
        blockType = NORMAL_BLOCK;
        //Serial.println("skip");
         return blockCount;
       }
      else if (checksum==PIXY_START_WORD_CC) //we've reacehd the beginning of the next frame - checking for 0xaa56
      {
        skipStart = true;
        blockType = CC_BLOCK;
        return blockCount;
      }
       else if (checksum==0)
         return blockCount;

      block = blocks + blockCount;

       for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
       {
        if (blockType==NORMAL_BLOCK && i>=5) // skip --if not an CC block, no need to consider angle
        {
         block->angle = 0;
         break;
        }
         w = getWord();
         sum += w; //sum = w + sum
         *((uint16_t *)block + i) = w; //converts block to interger value
       }
       if (checksum==sum)
         blockCount++;
       else
         printf("Pixy: cs error");

      w = getWord(); //when this is start of the frame
      if (w==PIXY_START_WORD)
        blockType = NORMAL_BLOCK;
      else if (w==PIXY_START_WORD_CC)
        blockType = CC_BLOCK;
      else
         return blockCount;
     }
   }

   int increment_since_last_find = 0;

   void parseLargestValues(){ //parses the array for the two largest objects and sets the index variables accordingly
	   numblocks = getBlocks(2);
	   if(numblocks >= 1){
		   skip = false;
		   for(int i = 0; i <= numblocks; i++){
			   if(blocks[i].width * blocks[i].height >= max){
				   secMax = max;
				   secMaxIndex = maxIndex;
				   max = blocks[i].width * blocks[i].height;
				   maxIndex = i;
				   //std::cout << blocks[i].x << std::endl;
			   }
			   else if(blocks[i].width * blocks[i].height > secMax){
				   secMax = blocks[i].width * blocks[i].height;
				   secMaxIndex = i;
				   //std::cout << blocks[i].x << std::endl;
			   }
		   }
	   }
//	   else if(numblocks < 1){
//		   skip = true;
//	   }
   }

   void averageAxis(){ //averages the two array objects at the indexes set by parseLargestValues()
	   parseLargestValues();
	   x1 = blocks[maxIndex].x;
	   x2 = blocks[secMaxIndex].x;
	   if(numblocks <= 1){
		   average = blocks[maxIndex].x;
		   badBlock = true;
	   }
	   else{
	   average = (x1 + x2) / 2;
	   badBlock = false;
	   }
   }

   void skipParse(){
	   numblocks = getBlocks(2);
	   average = (blocks[0].x + blocks[1].x) / 2;
   }

   int tracking(){ //tells whether the averaged values are to the left, right, or center
	   averageAxis();
	   //skipParse();
	   if(skip){
		   return -2;
	   }
	   else{
		   if(average > 171){ //originally 161 03/22/2017 144
			   return 1;
	   	   }
	   	   else if(average < 169){ //originally 159 03/22/2017 142
	   		   return -1;
	   	   }
	   	   else if(!badBlock) {
	   		   return 0;
	   	   }
	   	   else{
			   return 2;
	   	   }
	   }
   }

   float TrackingPID(){
	   float kpa = 0.009;
	   averageAxis();
	   e = ((145 - (average - 160)) * .01) + .12;
	   std::cout << "e= " << e << std::endl << "average = " << average << std::endl;
	   if(e < 0.13 && e > -0.13){
		   return 2000;
	   }
	   return e;
   }
};

