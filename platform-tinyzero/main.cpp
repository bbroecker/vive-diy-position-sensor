// Including Arduino.h is required for using Serial functions
#include <memory>
#include "Arduino.h"
//#include <memory>
#include "input_tim.h"
#include "car_control/bluetooth_communication.h"
#include "car_control/motor_controller.h"
#include "primitives/workers.h"
#include "pulse_processor.h"
#include "geometry.h"
#include "data_frame_decoder.h"
#include "i2c_communication.h"

class GeometrySerial : public Consumer<ObjectPosition> {

    void consume(const ObjectPosition& f) {
      //  SerialUSB.println("LET CONSUME!!");
//    DataChunkPrintStream printer(this, f.time, node_idx_);
   // SerialUSB.printf("OBJ%d\t%u\t%d\n", f.object_idx, f.time.get_value(msec), f.fix_level);
    if (f.fix_level >= FixLevel::kStaleFix ) {

        SerialUSB.println("GOT POSE");
        SerialUSB.print("x: ");
        SerialUSB.print(-f.pos[2], 4);
        SerialUSB.print("y: ");
        SerialUSB.print(-f.pos[0], 4);
        SerialUSB.print("z: ");
        SerialUSB.print(f.pos[1], 4);
        SerialUSB.print(" ob id: ");
        SerialUSB.print(f.object_idx, 4);
        SerialUSB.println();
    //   SerialUSB.printf("\t%.4f\t%.4f\t%.4f\t%.4f \t%d\n", f.pos[0], f.pos[1], f.pos[2], f.pos_delta, f.object_idx);
        //pos_measurement.x = -f.pos[2];
       // pos_measurement.y = -f.pos[0];
        //pos_measurement.z = f.pos[1];
        //last_update = millis();
        //if (!stateEstimatorHasPositionMeasurement())
        {
           // estimatorKalmanEnqueuePosition(&pos_measurement);
        }

        if (f.q[0] != 1.0f) {  // Output quaternion if available.
     //       Serial.printf("\t%.4f\t%.4f\t%.4f\t%.4f", f.q[0], f.q[1], f.q[2], f.q[3]);
        }
    }
    //Serial.printf("\n");
}


};

class AngleCon : public Consumer<SensorAnglesFrame>
{



void consume(const SensorAnglesFrame& f) {
    //uint32_t time = f.time.get_value(msec);

    // Print each sensor on its own line.
    for (uint32_t i = 0; i < f.sensors.size(); i++) {
        //DataChunkPrintStream printer(this, f.time, node_idx_);
        const SensorAngles &angles = f.sensors[i];


       // printer.printf("ANG%d\t%u\t%d", i, time, f.fix_level);
       SerialUSB.print(i);
       SerialUSB.print(": ");
        for (uint32_t j = 0; j < num_cycle_phases; j++) {
            //printer.printf("\t");
            if (f.fix_level == FixLevel::kCycleSynced && angles.updated_cycles[j] == f.cycle_idx - f.phase_id + j){
            //    printer.printf("%.4f", angles.angles[j]);
                SerialUSB.print(angles.angles[j]);
                //SerialUSB.print(j);
            }
            SerialUSB.print(" | ");
        }
        SerialUSB.print("\n");
    }
    //SerialUSB.println("angle");
}

};


class ConsumeDataFrame : public Consumer<DataFrame>
{

    void consume(const DataFrame& frame)
    {
        const DecodedDataFrame *df = reinterpret_cast<const DecodedDataFrame *>(&frame.bytes[0]);

        SerialUSB.print("current mode: ");
        SerialUSB.print(df->mode_current);
        SerialUSB.println();
    }

};

class ConsumeDebug : public Consumer<DebugString>
{

    void consume(const DebugString& debug_txt)
    {

        SerialUSB.print("debug text: ");
        for (int i = 0; i < 3; i++){
            SerialUSB.print(debug_txt.data[i]);
            SerialUSB.print(" ");
        }
        SerialUSB.println();
    }

};


std::unique_ptr<Pipeline> pipeline;
::Vector<BaseStationGeometryDef, 2> base_station_defs;
GeometryBuilderDef builder_def_1;
GeometryBuilderDef builder_def_2;
GeometryBuilderDef builder_def_3;
GeometryBuilderDef builder_def_4;


// the setup routine runs once when you press reset:
void setup() {

    BaseStationGeometryDef base_def_1 = {.mat = {-0.260703, 0.393942, -0.881387,  -0.079763, 0.901048, 0.426322 , 0.962118, 0.181445, -0.203484}, .origin = {-1.369170, 1.995489, -0.555100}};
    BaseStationGeometryDef base_def_2 = {.mat = {-0.311722, -0.229114, 0.922136, 0.022549, 0.968436, 0.248240, -0.949906, 0.098175, -0.296717}, .origin = {1.672622, 1.944626, -0.599123}};
    base_station_defs.push(base_def_1);
    base_station_defs.push(base_def_2);


    auto consumer = new GeometrySerial();
    auto consumer_angle = new AngleCon();
    auto consumer_debug = new ConsumeDebug();
    auto data_frame_decoder = new DataFrameDecoder(1);
    auto data_frame_consumer = new ConsumeDataFrame();
    data_frame_decoder->pipe(data_frame_consumer);
    pipeline = std::make_unique<Pipeline>();
    auto pulse_processor = pipeline->add_back(std::make_unique<PulseProcessor>(4));
    auto i2c_sender = pipeline->add_back(std::make_unique<I2CPoseSender>(8));
    pulse_processor->Producer<SensorAnglesFrame>::pipe(consumer_angle);
    pulse_processor->Producer<DataFrameBit>::pipe(data_frame_decoder);

    InputDef input_def_1 {4, false, InputType::kTimer, 0}; //PA09 --> Pin3 on Tiny and Pin5 on XIAO
    auto input_node_1 = std::make_unique<InputTimNode>(0, std::move(input_def_1));
    auto node = pipeline->add_front(std::move(input_node_1));
    node->pipe(pulse_processor);
    SensorLocalGeometry sensor_1 = {.input_idx=0, .pos = {0, 0, 0}};
    builder_def_1.sensors.push(sensor_1);
    auto geo_builder_node_1 = pipeline->add_back(std::make_unique<Point2DGeometryBuilder>(0, builder_def_1, base_station_defs, 0.6));
    pulse_processor->Producer<SensorAnglesFrame>::pipe(geo_builder_node_1);
    geo_builder_node_1->Producer<ObjectPosition>::pipe(consumer);




    InputDef input_def_2 {10, false, InputType::kTimer, 1};
    auto input_node_2 = std::make_unique<InputTimNode>(1, std::move(input_def_2));
    auto node_2 = pipeline->add_front(std::move(input_node_2));
    node_2->pipe(pulse_processor);
    SensorLocalGeometry sensor_2 = {.input_idx=1, .pos = {0, 0, 0}};
    builder_def_2.sensors.push(sensor_2);
    auto geo_builder_node_2 = pipeline->add_back(std::make_unique<Point2DGeometryBuilder>(1, builder_def_2, base_station_defs, 0.6));
    pulse_processor->Producer<SensorAnglesFrame>::pipe(geo_builder_node_2);
    geo_builder_node_2->Producer<ObjectPosition>::pipe(consumer);


    InputDef input_def_3 {11, false, InputType::kTimer, 2};
    auto input_node_3 = std::make_unique<InputTimNode>(2, std::move(input_def_3));
    auto node_3 = pipeline->add_front(std::move(input_node_3));
    node_3->pipe(pulse_processor);
    SensorLocalGeometry sensor_3 = {.input_idx=2, .pos = {0, 0, 0}};
    builder_def_3.sensors.push(sensor_3);
    auto geo_builder_node_3 = pipeline->add_back(std::make_unique<Point2DGeometryBuilder>(2, builder_def_3, base_station_defs, 0.6));
    pulse_processor->Producer<SensorAnglesFrame>::pipe(geo_builder_node_3);
    geo_builder_node_3->Producer<ObjectPosition>::pipe(consumer);

    InputDef input_def_4 {2, false, InputType::kTimer, 3};
    auto input_node_4 = std::make_unique<InputTimNode>(3, std::move(input_def_4));
    auto node_4 = pipeline->add_front(std::move(input_node_4));
    node_4->pipe(pulse_processor);
    SensorLocalGeometry sensor_4 = {.input_idx=3, .pos = {0, 0, 0}};
    builder_def_4.sensors.push(sensor_4);
    auto geo_builder_node_4 = pipeline->add_back(std::make_unique<Point2DGeometryBuilder>(3, builder_def_4, base_station_defs, 0.6));
    pulse_processor->Producer<SensorAnglesFrame>::pipe(geo_builder_node_4);
    geo_builder_node_4->Producer<ObjectPosition>::pipe(consumer);


    pipeline->start();
}

// the loop routine runs over and over again forever:
void loop() {
    //SerialUSB.println(";oop");
     pipeline->do_work(Timestamp::cur_time());
}
