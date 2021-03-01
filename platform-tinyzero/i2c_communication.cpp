#include "i2c_communication.h"
#include <Wire.h>
#include <Arduino.h>


I2CPoseSender* I2CPoseSender::senderSingleton=0;

I2CPoseSender::I2CPoseSender(int i2c_address) : pose_valid_(true), last_pose_{20.,10.,10., false}
{
    i2c_address_ = i2c_address;
}

void I2CPoseSender::consume(const ObjectPosition& f) {
//  SerialUSB.println("LET CONSUME!!");
//    DataChunkPrintStream printer(this, f.time, node_idx_);
// SerialUSB.printf("OBJ%d\t%u\t%d\n", f.object_idx, f.time.get_value(msec), f.fix_level);
    if (f.fix_level >= FixLevel::kStaleFix ) {
          /*  SerialUSB.println("GOT POSE");
            SerialUSB.print("x: ");
            SerialUSB.print(-f.pos[2], 4);
            SerialUSB.print("y: ");
            SerialUSB.print(-f.pos[0], 4);
            SerialUSB.print("z: ");
            SerialUSB.print(f.pos[1], 4);
            SerialUSB.print(" ob id: ");
            SerialUSB.print(f.object_idx, 4);
            SerialUSB.println();
        */
            last_pose_.x = -f.pos[1];
            last_pose_.y = -f.pos[0];
            pose_valid_ = true;
            if (f.q[0] != 1.0f) {  // Output quaternion if available.
            }
    }
}

void I2CPoseSender::onRequest(){
    if(senderSingleton)
        senderSingleton->sendPose();
}

void I2CPoseSender::sendPose(){

   // byte test[] = "hello\n";
    Wire.write((byte *)&last_pose_, sizeof(struct Pose2D));
    //Wire.write((byte *)test, sizeof(test));
    //Wire.write("hwllo oh yeah!!!\n");
    //SerialUSB.println("sendPose");
    //Wire.write("hello\n");
//   SerialUSB.print(last_pose_.valid);
 //   SerialUSB.print(" ");
  //  SerialUSB.print(sizeof(struct Pose2D));
   // SerialUSB.print("\n");
    pose_valid_ = false;

}

void I2CPoseSender::do_work(Timestamp time){
}


void I2CPoseSender::start()
{
    if (senderSingleton){
        SerialUSB.println("WARNING: I2CPoseSender has min 2 instances, it's a singleton.");
    }

    senderSingleton = this;
    Wire.begin(i2c_address_);
    Wire.onRequest(I2CPoseSender::onRequest);


}
