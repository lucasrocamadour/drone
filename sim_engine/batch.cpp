#include "engine.h"

#include <fstream>
#include <stdexcept>

using namespace std;

const char* BATCH_SHAPE_PATH = "output/output_shape_batch.jsonl";
const char* BATCH_TELEMETRY_PATH = "output/output_telemetry_batch.jsonl";
const char* BATCH_CONDITIONS_PATH = "input/input_conditions_batch.json";

void write_shape_file(Drone& D1, const char* shape_path){
    ofstream shape_file(shape_path);

    shape_file << "{";
    shape_file << "\"body_size\": "
               << "[" << 0.2 << "," << 0.2 << "," << 0.05 << "], ";
    shape_file << "\"rotor_positions\": "
               << "["
               << "[" << D1.rotor1.position_body.x << "," << D1.rotor1.position_body.y << "," << D1.rotor1.position_body.z << "],"
               << "[" << D1.rotor2.position_body.x << "," << D1.rotor2.position_body.y << "," << D1.rotor2.position_body.z << "],"
               << "[" << D1.rotor3.position_body.x << "," << D1.rotor3.position_body.y << "," << D1.rotor3.position_body.z << "],"
               << "[" << D1.rotor4.position_body.x << "," << D1.rotor4.position_body.y << "," << D1.rotor4.position_body.z << "]"
               << "]";
    shape_file << "}";
    shape_file.close();
}

int main(int argc, char** argv){
    const char* conditions_path = BATCH_CONDITIONS_PATH;
    const char* telemetry_path = BATCH_TELEMETRY_PATH;
    const char* shape_path = BATCH_SHAPE_PATH;

    if(argc == 4){
        conditions_path = argv[1];
        telemetry_path = argv[2];
        shape_path = argv[3];
    } else if(argc != 1){
        throw runtime_error("usage: batch.bin [conditions_path telemetry_path shape_path]");
    }

    Drone D1;
    gyro G1;
    accelerometer A1;
    barometer B1;
    gps GPS1;
    magnometer M1;

    write_shape_file(D1, shape_path);

    ofstream telemetry_file(telemetry_path);
    telemetry_file.close();

    run_sim_batch(
        D1,
        G1,
        A1,
        B1,
        GPS1,
        M1,
        telemetry_path,
        conditions_path
    );

    return 0;
}
